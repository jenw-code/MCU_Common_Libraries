# Coding Standards (AI)

The coding standards Claude works to on every project: the standing rules, followed by further lessons agreed along the way.

## The master rule: code must show intent

**KISS: keep it simple, stupid.** Read the code cold, without context. If it only makes sense with an explanation, it isn't clear yet. Clarity and efficiency are the goals; "less code" and compactness never are. Clear code often ends up smaller, but as a result of clarity, never as the aim.

1. **Comments explain intent, never functionality.** The code shows what it does; a comment gives the real reason a line exists. No side effect given as the reason, no obvious purpose. If a name already shows the intent, no comment. Plain English, wrapped at about 85 characters.
2. **Density of logic.** Don't pack several pieces of logic into one expression; give each its own simple, named step. A single guard with one outcome is fine. But don't name a value that's only used once, directly below: a `bool` set and then tested on the next line adds a name without adding meaning, so the condition goes straight in the `if`.
3. **No clever code.** Never bend a language feature to fit a pattern. Never turn a yes/no question into arithmetic. Write a check in the same terms as its question. Group conditions with the same result into one `if`; keep separate decisions as separate steps.
4. **Variables.** Declare each one right before the code it relates to. One meaning per variable. A "prev" copy is only for restoring. A default must be honest: an unread default never carries a meaning. A function that sets a state sets its own starting value (e.g. Unknown) at the top, rather than relying on the member's default and on being called only once: that's a hidden assumption.
5. **Buffer indexes.** Named `constexpr` indexes with one comment on the offset calculation, never an inline `buf[REG_A - REG_B]`.
6. **A constant's type must never subtly affect a calculation.** The constant has the type that fits its value; the calculation casts it where needed.
7. **Never shrink a type without a range table** covering every value and intermediate.
8. **Names show purpose.** A helper used by one feature says so. Bools never take an "is" prefix. A function's name gives its intent; its parameters give the flavour of how. Functions with the same name do the same thing. A variable that only changes another's type is named after it plus the type (`floatX`).
9. **A `bool` return must be trustworthy** and cover everything the call does. Return false for misconfiguration, but never add bus traffic to a hot path; cache the state instead.
10. **No magic numbers**, except a literal used once where its context makes it clear. A value used in one function is a local `constexpr`; class-level constants sit together in one block.
11. **Use language features correctly.** Qualifiers must be truthful. Never derive a value in a way that hides what it means.
12. **If the code relies on something, validate it.** A comment isn't a safeguard. `static_assert` at compile time, otherwise a runtime check. Better still, design the assumption out.
13. **Where a validation goes:** next to the shared declaration if several functions rely on it, otherwise first in the function that relies on it. Never act and then check. Validate once, at the entry point; don't guard what's already guaranteed.
14. **Never do work that can't ever be used.**
16. **The code's shape mirrors its intent.** Write it the way you'd explain it: "if the CRC is wrong, report it; otherwise, if the type is unknown, report that; otherwise, deliver the message" becomes exactly that chain. If the code's shape tells a different story (for example, working out every check up front when the first failure stops everything), the intent is lost, however correct the result. Where the code can't easily show its intent, a comment states it. Show the logic clearly even where it may be disagreed with: clear logic can be discussed and corrected, but hidden logic can't.
15. **Don't be lazy with headers.** Every function documents every `@param` and `@tparam` (what it is, units, range), whatever its scope.

## C++ / embedded library conventions (PlatformIO Arduino, 32-bit MCUs)

- **Get rid of every warning by removing its cause, not by silencing it.** Warnings left in a build are noise that hides the ones that matter. For example, if the core or board provides a macro that says whether a feature exists (such as `RP2350_PSRAM_CS`), use a simple `#if` on it to leave out the code that causes the warning.
- **No complex conditional compilation.** A single, simple `#if` / `#else` (for example, testing whether the board provides a feature) is fine. What's ruled out is a file of nested or combined `#if` conditions, which makes a tangle of behaviour nobody can follow. Vary real behaviour through OO or templates.
- **Floats:** none without a very good reason; use fixed-point scaling. Where floats make the code much clearer at a low call rate, use them with minimal scope, and convert explicitly (`static_cast<float>`).
- **Template parameters must earn their place** through compile-time code removal or a `static_assert`; otherwise use a runtime parameter, to avoid flash bloat.
- **`if constexpr` enum chains** end with `else return false;`; values needing no action get an explicit empty, commented branch; multi-value conditions go one per line.
- **Register bit maps:** one constant per bit, `(1 << n)`, from bit 0 up, reserved and fixed bits included, datasheet names. A multi-bit field is a single binary literal. Shift constants in their own block. `_MASK` only for a real mask.
- **Binary literals** (`0b…`, no leading zeros) for bitwise or bit-mapping configuration.
- **Names follow the datasheet** exactly in register and bit mapping. Everywhere else, sensible unabbreviated words. The exception is an abbreviation so common it loses no meaning (for example "Comms"). Never cite page or table numbers. Millisecond names end in `MILLIS`. A name says what the value is, never just its unit (`rate`, not `hertz`); the unit goes in the comment. Verbs:
  - "populate": a collection filled once that never changes.
  - "initialise": a value that will change later.
  - "read": from the device, even for a fixed property.
  - "parse": breaking down a data format from bytes already held, such as a whole frame. You parse XML.
  - "pack" and "unpack": byte manipulation, putting a value into bytes or taking it out of them. You unpack a
    stream of bytes.
  - "pack" / "unpack": one value into bytes, and back again (`packUInt32`, `unpackUInt32`).
  - "get": only a cached value or a status.
  - "enable…": only if the call actually makes the thing work; otherwise "set…".
  - `reset()`: the whole device.
- **Include everything from the datasheet.** Guard unusable items so an omission isn't ambiguous.
- **Libraries must be consistent** with each other in style, API and optimisation:
  - `startWire()` is one `&&` chain with each step's intent commented.
  - `quickSetup<...>()` is for devices that need a lot of configuration at start-up because their defaults wouldn't work in most cases. It returns `bool`, chains its setters with `&&`, and its defaults suit the most common use, not just the reset values. A device that works out of the box doesn't need one.
  - Helpers only change settings that can be reversed individually.
  - Self-tests are destructive and say so; cached state stays correct and test-only modes are turned off at the end.
  - Struct constants use designated initializers, one field per line. Arrays are initialised at declaration.
  - In the .cpp, dependencies come first, and the header follows the .cpp order.
  - Read, compare, and write only if the value changed. Cached device state with guard clauses that `return false`. An enum's 0 value is its default, so it must reflect reality: `Unknown = 0` when the state isn't known at the start, or the real starting state when it is. No STL. Timeouts use `millis()`.
  - Debug prints use the shared `logStr`/`logValue`, not `Serial` directly. Their string literals use `F()`: libraries aren't specific to one MCU, and on cores without memory-mapped flash it keeps the strings out of RAM.
- **Judge performance by how often each function is called.** A piece of code's priority for tuning is its average time multiplied by how often it runs.
- **Rare paths, such as error handling, get the simplest code.** Their frequency makes their cost negligible, so don't add work or extra variables to make a rare case perfect.
- **Criticality decides the level of checking.** Ask: does it matter if this didn't take effect? If not, or if the failure would soon show itself, don't add a check for it. A check nothing else in the code does is also inconsistent.
- **MCU-specific code says so in its name:** an MCU prefix on its folder, file and class. Code for different MCUs isn't bunched together in one folder.

## Further lessons

### Vocabulary and naming
- **One word per meaning, across the whole file and its family.** Never a mixture of words for the same thing. If a different word is preferred, everything changes to it. Once a thing has a name in the code, comments use that same name.
- **An amount of data is always a "size", in bytes,** whether it's a buffer's capacity or what the buffer holds. Never "length". "Count" is only for a quantity of something that isn't a collection, such as retries. Names used from outside code, such as a datasheet, keep their own word.
- **"on…" names an event; "handle…" names the function that deals with it.** For example, `onFrameReceived` is the event a caller can hook, and `handleFrameChange` is the code that responds to an interrupt.
- **Opposite pairs:** `start`/`stop`, `begin`/`end`, `setup`/`teardown`. A public function and the helper it calls use the same verb.
- **"Setup" means configuring something so it can be used**, including hardware that already exists. Prefer `setup`/`teardown` to `create`/`destroy`.
- **Avoid words with more than one meaning in context.** For the hardware at the other end of a link, "device", not "module", which also means software.
- **A new enum value goes where it logically fits**, not simply at the end. For example, errors are ordered as they can occur, so a new one sits beside the errors detected at the same stage. An unknown state always goes first.
- **Type names have no `_t` suffix**, whether class, struct, enum or template parameter. The exception is an alias of a type that already ends in `_t` (for example `using Distance_t = uint32_t;`).
- **A yes/no choice is a clearly named `bool`, not a two-value enum** (e.g. `TWO_STEP_PAYLOAD_IN_PSRAM`). Use an enum only when there are more than two options, or the two options aren't a yes and a no.
- **Read names as a stranger would.** A name that needs explaining isn't right yet. A size is named in the unit a reader expects (bytes, not a power-of-two exponent).- **A class's name and doc say what it's for, and what it can't be used for.** Ask "what would someone assume from this?".

### Comments and headers
- **A detail that's defined once in code isn't repeated in comments.** For example, once a constant says which level means "active", comments elsewhere don't mention levels; otherwise they go stale when the detail changes.
- **Don't make a comment more specific than its intent needs.** Naming the exact function or detail that relies on a line makes the comment go out of date, or mislead, as soon as the code changes a little. Give the general purpose, with a current use as an example: "Kept to aid in sending data (e.g. used to check when sending has completed)". It names no functions, and the "e.g." shows a use without limiting the value to it, so the comment stays true as uses are added. A comment so general that it only states the obvious ("needed after setup") is pointless.
- **A comment must read one way only.** A clipped opening such as "Last, so…" can be read as narration ("and lastly") rather than as the reason. State the reason in full: "Attached once everything it uses is set up, so…".
- **Comments must be true on every path the code runs.** Check every word, especially "again", "always" and "any". A simple true comment beats an explanatory one that claims something the code can't guarantee.
- **A subtle call gets a comment stating the structure** it builds (for example, a `memset` followed by assigning the fields that matter).
- **Keep chunks of meaning together.** Break lines between ideas, not mid-idea. Don't wrap a short example; only wrap a very long one.
- **A function's header gives its purpose, not its mechanism.** Say what it's for ("tell the receiver a frame is about to be sent"), not how it does it ("lower the line"). The code shows the how.
- **Calculation functions give an example** of an input and its output in their header.
- **An explanatory comment goes above the line**, not as a side note, and leads with the intent.
- **Identical lines get identical comments.**
- **A very pertinent, subtle point starts with "Note:"** to draw the developer's eye to it. Kept for those points only, so it keeps its weight.
- **Code doesn't explain the schematic.** Circuit details, such as an external pull-up on a pin, belong in the hardware design, not in comments on the code that uses them.
- **Each line of a multi-line brief starts with its own `/// @brief`.**

### Writing code
- **Write conditions positively, in the terms of the intent.** Name every accepted case, so the check fails closed, rather than rejecting the wrong cases, which fails open.
- **One truth for data mapping.** A frame or register layout is defined once (the index constants), and code builds or reads it through those indexes, never by position.
- **No implicit truths.** Every constant is used only for what its name says. The test: "if I change this value, is the code still predictable?"
- **A value used once, whose name adds nothing, goes straight into the call.** Declaring it first hides that it's used only once. Name it only when the name explains something the expression doesn't.
- **An early `return` is for a failure or guard.** When two branches are both normal outcomes, use `if`/`else`, so neither reads as an error.
- **Never `while (true)`.** Put the real condition in the `while`.
- **Where a line sits shows intent.** A line inside a loop says it may change on each pass, so anything that stays the same for the whole loop goes before it.
- **Wrap ring indexes with increment-and-if, not modulo**, as modulo is slow on 8-bit MCUs. The exception is a compile-time constant divisor, which the compiler optimises.
- **Initialise every struct member** (`= {}`), since C structs can hold garbage.
- **Memory a DMA uses in the background is permanent, not local.** A local buffer only works while every caller remembers to wait before returning, and forgetting gives a transient bug that depends on what runs next. A permanent buffer makes the one remaining rule explicit and easy to see: don't write to it while its transfer is running. Waits are then only for ordering (e.g. a line changes only after the last byte has gone), never to keep memory alive.
- **Public functions take references, not pointers.** A function that produces a result returns `bool` and fills a reference out-parameter, changed only on success.
- **A utility class** is static functions with a deleted constructor.
- **Only add something if there's a plan to use it.** A plausible future use isn't enough.

### Structure and OO
- **The base class defines the interface.** Each derived class may implement it in its own way, or the base may implement it once. The test: a base-class variable set to either derived class, and the code still works.
- **Classes with the same functionality have the same structure and behaviour.** Compare them side by side after every change.
- **Duplication means the design has a flaw.** Shared logic lives once; the classes that differ supply only what genuinely differs. The exception: a small, simple piece (a few lines) can be repeated when sharing it would put it somewhere that confuses that place's purpose.
- **Don't split one flow across classes.** A developer shouldn't have to jump between classes to follow one flow. Keep the whole flow in one place, and have the others supply self-contained pieces.
- **Choose inheritance or a template engine by which class drives.** Virtual methods are fine; what hurts clarity is calls going back and forth between base and derived. When the derived class drives and uses what it inherits, inheritance fits. When the base class drives and the derived classes only supply helper methods for it to call, make those helpers an engine the driving class takes as a template parameter, built with its own config and handed in. The flow then reads top to bottom in one class. It's a judgement made case by case, not a fixed rule. One measure: if the derived classes could easily be replaced by a utility class, the inheritance probably isn't needed.
- **A call names the action, so the callee doesn't infer it from a value.** `startSendFrame()` tells the callee what's happening; `writeLine(LOW)` makes it work out what a level means.
- **A driver decides *when*; an engine knows *how*.** When one class runs a flow on top of hardware-specific code, the driving class owns every decision (when a frame starts, when to check), and the engine only carries actions out on its hardware. Like a car: the driver turns the headlights on; the electronics know how to power the bulb.
- **Think about each function's responsibility** to decide where logic belongs. Logic in the wrong place sends a false message about the code around it.
- **Don't conflate separate concerns.** If a function is hard to name, it's probably doing two things. A function earns its place when its name is clear and its purpose is single.
- **Keep independent concerns independent.** For example, a transport must not decide device behaviour.
- **A general-purpose helper holds only what every caller needs.** Anything specific to one caller goes in that caller's class, which builds on the helper. For example, a protocol's handshake line doesn't belong in a UART helper.
- **Libraries that do the same job on different MCUs have the same structure:** the same config struct, result structs, function names and class form. A developer who knows one MCU's version can then read any other. Their types and internals can differ, since only one MCU's files are compiled into a build.
- **Structure in proportion to functionality.** A whole class to understand, for a small piece of logic, is disproportionate.
- **A struct is for data.** Constants don't go in data structs, and a struct's fields shouldn't be half unused depending on a "kind".
- **Describe similar things the same way.** If one half of a pair is described one way, the other half is described the same way.
- **One source of truth.** If a value must be stated in two places, one is validated against the other. Better still, one constant drives every place that must agree, and the types make any disagreement a compile error.
- **Every base class with virtual functions has a public virtual destructor.** Deleting an object through a base pointer is normal OO.
- **A class frees only the resources it created.** Resources given to it are the giver's to free.
- **Whatever setup gives you, you give back to teardown.** The caller already knows the values it chose, so they don't need to be stored in the result structs.
- **Configs are `constexpr`-friendly:** hold indices, not pointers that come from casts.
- **Setup takes the resource explicitly** rather than picking one itself, so it fits a planned allocation.
- **Use a distinct type to force a required step.** If something must be set up a particular way first, give the result its own type and take that type, even if it duplicates another struct's fields. The compiler then enforces what a comment could only ask for, and the signature itself tells the developer what the class requires.
- **Things configured separately are passed separately.** Don't bundle independent things into one struct or call; kept apart, they can be mixed.
- **Class sections go private, then protected, then public.** Within each section, compile-time parts (type definitions and `constexpr` constants) come first, then runtime parts. This follows the natural order of dependency.
- **File order follows use:** what you give (the config), then what you get back, then the helpers that turn one into the other.
- **No complex web of dependencies** between libraries.
- **Keep one goal, and judge every decision against it.** Don't add complexity for a goal that isn't wanted, such as genericity nobody needs.

### Validation and failure
- **Setup requirements are validated in `begin`**, so a developer can see everything the setup needs in one place, and no check runs on every read.
- **A crash on a setup error is better than code that quietly carries on.** Don't add per-call checks to protect against a developer's incorrect setup.
- **A `bool` return that can never be false gives the caller nothing:** make it `void`.

### Interrupts
- **ISR code must be quick.** Anything that waits, delays or takes a lock belongs in the main loop; the ISR at most marks work as due.

### Judging a design
- **A library succeeds when it's simple to use, not just when it works.** Start a design from what a developer writes to use it, and keep that as simple as the job allows. Complexity a user has to deal with, such as nested templates in a declaration, is a design failure even if the code works.
- **Don't rely on a shared hardware resource that can't report it's in use.** Without a busy flag or a claim, another user can take it over silently and corrupt the result.
- **Agree the overall approach first, at a high level:** the responsibilities and how the pieces fit. Low-level detail comes once the structure is agreed.
- **The first solution isn't necessarily the right one.** Compare alternatives before presenting one, and challenge every solution with fresh eyes.
- **Never show a solution you're not happy with.** Keep thinking until it's one you'd stand behind, rather than presenting it with caveats.
- **Check assumptions before using them** to answer a question.
- **Raise significant side effects and trade-offs before making a change**, including what the change gives up.
- **A "cost" is something given up against the goal**, never the work of achieving the goal.
- **Never state an option as a requirement.** "Needs" and "must" claim that leaving something out breaks something; if it only helps, say so. Don't overclaim benefits.
- **Ask only when there's a real compromise** the user would weigh. When the best answer is clear, make the choice and give its reason.
- **Don't add code that will burn us later**; something that fails at a distance, in code other than its own, is the worst kind.
- **After a change, read the whole class from top to bottom** as one piece, checking that names and verbs use one vocabulary.
- **Only give information that brings value.** Challenge every note before including it.
