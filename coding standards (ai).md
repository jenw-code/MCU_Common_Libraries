# Coding Standards (AI)

The coding standards Claude works to on every project: the standing rules, followed by further lessons agreed along the way.

## The master rule: code must show intent

**KISS: keep it simple, stupid.** Read the code cold, without context. If it only makes sense with an explanation, it isn't clear yet. Clarity and efficiency are the goals; "less code" and compactness never are.

1. **Comments explain intent, never functionality.** The code shows what it does; a comment gives the real reason a line exists. No side effect given as the reason, no obvious purpose. If a name already shows the intent, no comment. Plain English, wrapped at about 85 characters.
2. **Density of logic.** Don't pack several pieces of logic into one expression; give each its own simple, named step. A single guard with one outcome is fine.
3. **No clever code.** Never bend a language feature to fit a pattern. Never turn a yes/no question into arithmetic. Write a check in the same terms as its question. Group conditions with the same result into one `if`; keep separate decisions as separate steps.
4. **Variables.** Declare each one right before the code it relates to. One meaning per variable. A "prev" copy is only for restoring. A default must be honest: an unread default never carries a meaning.
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
15. **Don't be lazy with headers.** Every function documents every `@param` and `@tparam` (what it is, units, range), whatever its scope.

## C++ / embedded library conventions (PlatformIO Arduino, 32-bit MCUs)

- **No `#define` to change behaviour.** The preprocessor is only for constants; vary behaviour through OO.
- **Floats:** none without a very good reason; use fixed-point scaling. Where floats make the code much clearer at a low call rate, use them with minimal scope, and convert explicitly (`static_cast<float>`).
- **Template parameters must earn their place** through compile-time code removal or a `static_assert`; otherwise use a runtime parameter, to avoid flash bloat.
- **`if constexpr` enum chains** end with `else return false;`; values needing no action get an explicit empty, commented branch; multi-value conditions go one per line.
- **Register bit maps:** one constant per bit, `(1 << n)`, from bit 0 up, reserved and fixed bits included, datasheet names. A multi-bit field is a single binary literal. Shift constants in their own block. `_MASK` only for a real mask.
- **Binary literals** (`0b…`, no leading zeros) for bitwise or bit-mapping configuration.
- **Names follow the datasheet** exactly in register and bit mapping. Everywhere else, sensible unabbreviated words. Never cite page or table numbers. Millisecond names end in `MILLIS`. Verbs:
  - "populate": a collection filled once that never changes.
  - "initialise": a value that will change later.
  - "read": from the device, even for a fixed property.
  - "parse": from bytes already held.
  - "get": only a cached value or a status.
  - "enable…": only if the call actually makes the thing work; otherwise "set…".
  - `reset()`: the whole device.
- **Include everything from the datasheet.** Guard unusable items so an omission isn't ambiguous.
- **Libraries must be consistent** with each other in style, API and optimisation. `startWire()` is one `&&` chain with each step's intent commented. Every library has a `quickSetup<...>()`. Helpers only change settings that can be reversed individually. Self-tests are destructive and say so; cached state stays correct and test-only modes are turned off at the end. Designated initializers for struct constants. In the .cpp, dependencies first, and the header follows the .cpp order.
- **Judge performance by how often each function is called.**

## Further lessons

### Vocabulary and naming
- **One word per meaning, across the whole file and its family.** Never a mixture of words for the same thing. If a different word is preferred, everything changes to it.
- **Opposite pairs:** `start`/`stop`, `begin`/`end`, `setup`/`teardown`. A public function and the helper it calls use the same verb.
- **"Setup" means configuring something so it can be used**, including hardware that already exists. Prefer `setup`/`teardown` to `create`/`destroy`.
- **Avoid words with more than one meaning in context.** For the hardware at the other end of a link, "device", not "module", which also means software.
- **Read names as a stranger would.** A name that needs explaining isn't right yet. A size is named in the unit a reader expects (bytes, not a power-of-two exponent).

### Comments and headers
- **Keep chunks of meaning together.** Break lines between ideas, not mid-idea. Don't wrap a short example; only wrap a very long one.
- **Calculation functions give an example** of an input and its output in their header.
- **An explanatory comment goes above the line**, not as a side note, and leads with the intent.
- **Identical lines get identical comments.**

### Structure and OO
- **The base class defines the interface.** Each derived class may implement it in its own way, or the base may implement it once. The test: a base-class variable set to either derived class, and the code still works.
- **Classes with the same functionality have the same structure and behaviour.** Compare them side by side after every change.
- **Duplication means the design has a flaw.** Protocol logic lives once, in the base; derived classes supply only what genuinely differs (the transport).
- **A struct is for data.** Constants don't go in data structs, and a struct's fields shouldn't be half unused depending on a "kind".
- **Describe similar things the same way.** If one half of a pair is described one way, the other half is described the same way.
- **One source of truth.** If a value must be stated in two places, one is validated against the other.
- **Every base class with virtual functions has a public virtual destructor.** Deleting an object through a base pointer is normal OO.
- **A class frees only the resources it created.** Resources given to it are the giver's to free.
- **Whatever setup gives you, you give back to teardown.** The caller already knows the values it chose, so they don't need to be stored in the result structs.
- **Configs are `constexpr`-friendly:** hold indices, not pointers that come from casts.
- **Setup takes the resource explicitly** rather than picking one itself, so it fits a planned allocation.
- **File order follows use:** what you give (the config), then what you get back, then the helpers that turn one into the other.

### Validation and failure
- **Setup requirements are validated in `begin`**, so a developer can see everything the setup needs in one place, and no check runs on every read.
- **A crash on a setup error is better than code that quietly carries on.** Don't add per-call checks to protect against a developer's incorrect setup.
- **A `bool` return that can never be false gives the caller nothing:** make it `void`.

### Interrupts
- **ISR code must be quick.** Anything that waits, delays or takes a lock belongs in the main loop; the ISR at most marks work as due.

### Judging a design
- **The first solution isn't necessarily the right one.** Compare alternatives before presenting one, and challenge every solution with fresh eyes.
- **Check assumptions before using them** to answer a question.
- **Raise significant side effects and trade-offs before making a change**, including what the change gives up.
- **A "cost" is something given up against the goal**, never the work of achieving the goal.
- **Don't add code that will burn us later**; something that fails at a distance, in code other than its own, is the worst kind.
- **Only give information that brings value.** Challenge every note before including it.
