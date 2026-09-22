# Before the demos: the shape of a Chrono program

Not part of the talk. It is here because the talk assumes you have seen a
Chrono script before, and if you have not, this is the shortest one that does
something:

```bash
python demos/first_steps/hello_chrono.py
```

A ball falls onto a floor. Every other demo in this repository -- the HMMWV,
on soil, the quadruped, the robot arm -- is this same four-part shape with
more bodies in it:

| | |
|---|---|
| a **system** | owns the bodies, the constraints, and the clock |
| some **bodies** | mass, collision geometry, where they start |
| a **window** | draws whatever the system currently holds |
| a **loop** | draw, then advance time by one step |

The one thing it deliberately does NOT do is pace itself to the wall clock, so
it runs as fast as the machine allows. That is right for a batch job and wrong
the moment a person is watching, which is where Demo 1 starts.
