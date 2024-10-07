/*

Overview: This SuperCollider code opens a serial connection to an Arduino (that is reading 2 weight sensors, and looking for the precense of 4 shapes (via a voltage divider analog input)), and providing the data as a continual ascii stream where the values are in pairs of the sensor value and the shape character. e.g. 500e750f-1g-1h501e749f-1g-1h. The stream is parsed, and then the sensor values are stored, and used as parameters to control 4 Synths. If a shape is not connected, the volume of the Synth is set to 0.
Author: Mark Altosaar
https://github.com/MarkErik/pull-o-phone

Resources used for sample code, tutorials, inspiration from:
Tutorial on connecting to Arduino: "https://www.youtube.com/watch?v=_NpivsEva5o"
Overall Arduino resources: "https://www.youtube.com/@elifieldsteel" (same professor who made the connecting to Arduino tutorial above)
Modified the MoogBass from as the Synth for smallSquare: "https://github.com/loopier/synthdefs/tree/master"
Modified examples to create the Synth for circle: "http://composerprogrammer.com/teaching/supercollider/sctutorial/2.4 Modulation Synthesis.html"

//////////
Mapping of ascii letters and values from Serial port to Synths
(between Arduino and SuperCollider)

Why letters e,f,g,h? I had named my weight sensors A & B, and so I continued with the letter theme.
And initially I was working with the resistor values and had't placed them inside the shapes.

code - shape - resistor/sticker colour - voltage divider reading
e - small box - resistor value of 2.36K (Red Dot) will give analog Pin reading of: 188
f - large square box - resistor value of 6.19K (Blue Dot) will give analog Pin reading of:  378
g - medium cylinder - resistor value of 14.6K (Green Dot) will give analog Pin reading of: ~601-602
h - sandpaper small square box - resistor value of 35K (Purple Dot) will give analog Pin reading of: 801

If nothing is plugged in, the analag input pin will read 1023 (5V), send -1
///////////

*/


//List out what devices are connected to the serial port
//Pick the one that matches what the Arduino IDE shows, and use that in the next line
SerialPort.devices;

//Connect to the serial port of the Arduino
//NOTE: The serial monitor of the Arduino IDE needs to be closed
~port = SerialPort.new("/dev/tty.usbmodem14101", 115200);

//Start the SuperCollider server where we will load the Synths
s.boot;


//create an empty array to hold the characters coming in from the serial port
~charArray = [ ];
//read the characters coming from the serial port, and if they are numbers, store them in the array
//if come across a character, assign the number that is in the array to the corresponding letter variable
//if come across a '-', that means the next character is 1, (since that is the only negative number sent, and assign -1 to the array, and then based on the next letter encountered, assign -1 to the associated variable
~getValues = Routine.new({
	var ascii;
	{
		ascii = ~port.read.asAscii;

		if(ascii == $-,{
			~port.read.asAscii;
			~charArray = ~charArray.add(-1);
		});

		if(ascii.isDecDigit,{~charArray = ~charArray.add(ascii)});
		if(ascii == $e, {
			if(~charArray[0] == -1,
				{
					~val_E = -1;
				},
				{
					~val_E = ~charArray.collect(_.digit).convertDigits;
			});
			~charArray = [ ];
		});
		if(ascii == $f, {
			if(~charArray[0] == -1,
				{
					~val_F = -1;
				},
				{
					~val_F = ~charArray.collect(_.digit).convertDigits;
			});
			~charArray = [ ];
		});
		if(ascii == $g, {
			if(~charArray[0] == -1,
				{
					~val_G = -1;
				},
				{
					~val_G = ~charArray.collect(_.digit).convertDigits;
			});
			~charArray = [ ];
		});
		if(ascii == $h, {
			if(~charArray[0] == -1,
				{
					~val_H = -1;
				},
				{
					~val_H = ~charArray.collect(_.digit).convertDigits;
			});
			~charArray = [ ];
		});
	}.loop;
}).play;


//Test section to look at the values interpreted from the serial port
~val_E;
~val_F;
~val_G;
~val_H;

//Define Synth for the smallSquare
(
SynthDef(\smallSquare,{
	arg out= [0,1], freq = 150, amp = 0.5, gate=1, attackTime= 0.2, fenvamount=16, cutoff= 500, gain=1.0, pan=0.0, volume = 1;

	var osc, filter, env, filterenv, carrfreq;

	osc = Mix(Pulse.ar(freq.lag(0.05)*[1.0,1.001,2.0],Rand(0.45,0.5)!3,0.33));

	filterenv = EnvGen.ar(Env.adsr(attackTime,0.0,1.0,0.2),gate,doneAction:2);
	filter =  MoogFF.ar(osc,cutoff*(1.0+(fenvamount*filterenv)),gain);

	env = EnvGen.ar(Env.adsr(attackTime, \decayTime.kr(0.3), \sustainLevel.kr(0.9), \releaseTime.kr(0.2)),gate,doneAction:2);

	Out.ar(out,Pan2.ar((0.7*filter+(0.3*filter.distort))*env*amp*volume*0.75,SinOsc.kr(pan)));
}).add;
)

//Define Synth for the bigSquare
(
SynthDef.new(\bigSquare, {
	arg cutoff=600, volume=1;
	var sig;
	sig = Saw.ar([50,51]);
	sig = RLPF.ar(sig, cutoff.lag(0.02), 0.25,0.3);
	Out.ar(0,sig*volume*0.9);
}).add;
)

//Define Synth for the circle
(
SynthDef.new(\circle, {
	arg harmonicity = 4, volume = 1; //range 1 to 8 for harmonicity
	var sig, carrfreq, modfreq, modindex;

	carrfreq= 220;

	modindex = LFSaw.kr(1);

	modfreq= carrfreq*harmonicity;

	sig = SinOsc.ar(carrfreq+(SinOsc.ar(modfreq)*modfreq*modindex), 0.0,0.1);
	Out.ar([0,1],sig*volume);

}).add;
)

//Define synth for the sandpaper square
(
SynthDef.new(\sandy, {
	arg freq = 3000, volume = 1; //range from 3000 to 18000
	var sig;
	sig = PinkNoise.ar(SinOsc.kr(4));
	sig = LPF.ar(sig, freq);
	Out.ar([0,1],sig*volume*0.8);
}).add;
)

//register small square Synth with volume 0, and then begin Routine to read the value assigned from the port reading routine to apply to the Synth
~synth_E = Synth(\smallSquare, [\freq,~val_E.linexp(0,850, 150,350), \volume,0]);
(
~control_E = Routine.new({
	{
		if(~val_E.isNegative,{
			~synth_E.set(\volume, 0);
		},
		{
			~synth_E.set(\freq, ~val_E.linexp(0,850, 150,350), \volume, 1);
		});
		0.01.wait;
	}.loop;
}).play;
)


//register big square Synth with volume 0, and then begin Routine to read the value assigned from the port reading routine to apply to the Synth
~synth_F = Synth(\bigSquare, [\cutoff,~val_F.linexp(0,750, 600,1600), \volume,0]);
(
~control_F = Routine.new({
	{
		if(~val_F.isNegative,{
			~synth_F.set(\volume, 0);

		},
		{
			~synth_F.set(\cutoff, ~val_F.linexp(0,750, 600,1600), \volume, 1);
		});
		0.01.wait;
	}.loop;
}).play;
)



//register circle Synth with volume 0, and then begin Routine to read the value assigned from the port reading routine to apply to the Synth
~synth_G = Synth(\circle, [\harmonicity,~val_G.linexp(0,800, 1,8), \volume,0]);
(
~control_G = Routine.new({
	{
		if(~val_G.isNegative,{
			~synth_G.set(\volume, 0);
		},
		{
			~synth_G.set(\harmonicity,~val_G.linexp(0,800, 1,8), \volume, 1);
		});
		0.01.wait;
	}.loop;
}).play;
)


//register sandpaper square Synth with volume 0, and then begin Routine to read the value assigned from the port reading routine to apply to the Synth
~synth_H = Synth(\sandy, [\freq,~val_H.linexp(0,800, 3000,18000), \volume,0]);
(
~control_H = Routine.new({
	{
		if(~val_H.isNegative,
			{
				~synth_H.set(\volume, 0);

			},
			{
				~synth_H.set(\freq, ~val_H.linexp(0,800, 3000,18000), \volume, 1);
		});
		0.01.wait;
	}.loop;
}).play;
)

//
//For each running Synth, first stop the Routine (loop) that is changing the parameters
//Then release the Synth from the server
//
~control_E.stop;
~synth_E.free;

~control_F.stop;
~synth_F.free;

~control_G.stop;
~synth_G.free;

~control_H.stop;
~synth_H.free;


//Clean up any processes not attached, and still running on the server
s.freeAll;

//Close the connection to the Arduino
~port.close;

//Stop the SuperCollider server
s.quit;
