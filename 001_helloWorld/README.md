# 001_helloWorld

Hello World example using ITM in Cortex-M4
This code is from Embedded-C programming course by Fastbit Brain Academy.

To get this working,
	- Enable `SWV` (Serial Wire Viewer) in project debug config.

	- In debug perspective view, add `SWV ITM Data Console` view.

	- In `ITM Data Console View`, Configure trace -> ITM Stimulus Ports -> Select 0th bit.

	- Now click `Start Trace` and run the program to get "Hello World" in Data Console.

> Use `\n` in each printf() to print in the console.