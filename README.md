# turbo-ems
A bunch of this code comes from speeduino by Josh Stewart.

The board I developed is a drop in replacement for the Toyota Echo ECU. the goal being to plug in to the stock wiring harness, no cutting required.

Originally I planned to use speeduino without much modification. But once I started looking it over, I decided speeduino was too inefficient for my purposes. I would have started fresh, but I wanted to retain compatibility with tunerstudio.

Some of my changes/goals are:

-All new scheduler, using only one timer. This leaves all the other timers free to do other stuff. Also this works up to an arbitrary number of cylinders.

-All new decoders, each engine will have it's own decoders file.

-Configuration at compile time, rather than a bunch of if() statements. This is important because it is much more efficient. If a parameter is specific to an engine, it should be set at compile time.

-New units for angles based on unsigned 16bit integers. Now 2^16=720 degrees. This is very useful because integer roll-over allows us to add and subtract angles without checking if we've gone past 720 degrees.

-Brand new interrupt-driven PID algorithm. This should be much more efficient and based on fixed rather than floating point.

Future goals:

-Get entirely free of C++

-Do away with the arduino compiler and library.

-Create a make file.


