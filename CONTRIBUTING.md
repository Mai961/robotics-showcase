# Adding a module

This repository holds evidence, not algorithms: what a module is, what it produced, and numbers that
can be read off its recordings. The source that implements an algorithm (a solver, a filter, a
control law) stays in the private repositories, and so does any script that re-implements one.
Infrastructure (transport, interfaces, the motor layer) may appear as a short, simplified excerpt.

To add a module, write its README with the same eight sections as the other modules, in the same
order: What it does, Why it exists, How it works, Interfaces, How it was run, Replay, Status,
Credits (the two bridge modules add a shared "Why two bridges"). "How it works" is one short
paragraph that says what the module does and why the design was chosen: a competent engineer should
be able to explain the module from it, but not rebuild it. Leave out formulas, step-by-step
walkthroughs, constants, thresholds and gains.

"Replay" describes the recording: the source bag or log and the window cut from it, a table of its
topics or columns, what was changed (blurred, derived, copied to the window start), how to open it
with `ros2 bag play`, in Foxglove Studio and in the hosted viewer, and a few lines of Python that
read it without ROS. Keep only the topics the module reads or produces, blur every camera image
except the tags, and end the section with the license note: "Recordings © Zile Liao: they may be
viewed and analysed, not redistributed."

An analysis script goes into `<module>/replay/analyze_<name>.py`. It may only read what the system
logged: no solver, no filter, no control law, and nothing recomputed that the log already holds. It
takes `--log` (default: the bundled recording) and `--out` (default: `replay_out/` next to the
script), has a truthful `--help`, needs only the packages in `requirements.txt`, is deterministic,
writes only under `--out`, and runs in under 10 s. Paste its output into the module's README under
"### Analysis", commit its figures under `<module>/replay/replay_out/` (regenerated from the
`requirements.txt` environment), and add it to the root `README.md`'s "Run the analyses" block and
table and to `.github/workflows/analyze.yml`.

The `Status:` line says "recording + viewer + measured numbers" when the module has an MCAP
recording and an analysis script, "recording + measured numbers" or "recording + viewer" when it has
only one of the two, "infrastructure excerpt" for simplified infrastructure source, or "clip only
(website)" when only the project site shows it. Write any number the recordings do not contain as
"not measured". Take Credits from `git log --no-merges --format=%an -- <paths>` in the source
repository.

Before committing, run a secrets scanner (`gitleaks detect --source . --no-git`, or
`detect-secrets scan`) and `grep -rnIE 'password|passwd|token|secret|api_key|PRIVATE KEY|[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' .`,
and check every hit. Finally, add the module's row to the table, with its viewer link, and its node
to the diagram, in the root `README.md`.
