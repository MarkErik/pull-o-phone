SerialPort.devices;

~port = SerialPort.new("/dev/tty.usbmodem14101", 115200);

s.boot;


/*

Mapping of ascii letters and values from Serial port to Synths
(same between Arduino and SuperCollider)

e - small square box
f - large square box
g - medium cylinder
h - two-tone small square box

e - small box - resistor value of 2.36K (Red Dot) will give analog Pin reading of: 188
f - large square box - resistor value of 6.19K (Blue Dot) will give analog Pin reading of:  378
g - medium cylinder - resistor value of 14.6K (Green Dot) will give analog Pin reading of: ~601-602
h - two-tone small square box - resistor value of 35K (Purple Dot) will give analog Pin reading of: 801

if value is -1, means that the shape is unplugged, and the corresponding synth should be silent

*/

~charArray = [ ];
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

~val_E;
~val_F;
~val_G;
~val_H;
~val_G.isNegative;


(
SynthDef.new(\sawSmall, {
	arg cutoff=1000, volume=1;
	var sig;
	sig = Saw.ar([50,51]);
	sig = RLPF.ar(sig, cutoff.lag(0.02), 0.25,0.2);
	//sig = HPF(sig, 440);
	Out.ar(0,sig*volume);
}).add;
)

(
SynthDef.new(\sawBig, {
	arg cutoff=500, volume=1;
	var sig;
	sig = Saw.ar([25,26]);
	sig = RLPF.ar(sig, cutoff.lag(0.02), 0.25,0.2);
	Out.ar(0,sig*volume);
}).add;
)

(
SynthDef.new(\circle, {
	arg in_num = 10000, volume = 1;
	//low-pass filter with mouse-controlled cutoff frequency
	var sig;
	sig = PinkNoise.ar(0.25!2);
	sig = LPF.ar(sig, in_num);
	Out.ar(0,sig*volume);
}).add;
)

(
SynthDef.new(\mixed, {
	arg in_num = 0.5, volume = 1;
	//bandpass filter with center freq = 1 kHz and mouse-controlled rq, also using some rq math for BPF mul argument to compensate for lost amplitude
	var sig, rq;
	rq = in_num;
	sig = PinkNoise.ar(0.5!2);
	//sig = Saw.ar([60,61], 0.2);
	sig = BPF.ar(
		sig,
		1000,
		rq,
		1/rq.sqrt,
	);
	Out.ar(0,sig*volume);
}).add;
)

p = Synth(\circle, 16000);
p.free;

x = Synth(\sawSmall, 1000);
x.free;

y = Synth(\sawBig, 500);
y.free;



~val_A = 500;

//~synth_A = Synth(\saw, [\cutoff,~val_A.linexp(0,1000, 80,4000),\t_trig,~val_A]);

~val_E;
~val_E.isNegative;
//register Synth with volume 0
~synth_E = Synth(\sawSmall, [\cutoff,~val_E.linexp(0,1000, 80,4000), \volume,0]);
(
~control_E = Routine.new({
	{
		if(~val_E.isNegative,{
				~synth_E.set(\volume, 0);
			},
			{
			~synth_E.set(\cutoff, ~val_E.linexp(0,1000, 80,4000), \volume, 1);
		});
		0.01.wait;
	}.loop;
}).play;
)


~control_E.stop;
~synth_E.free;

//register Synth with volume 0
~synth_F = Synth(\sawSmall, [\cutoff,~val_F.linexp(0,1000, 80,4000), \volume,0]);
(
~control_F = Routine.new({
	{
		if(~val_F.isNegative,{
			~synth_F.set(\volume, 0);

		},
			{
			~synth_F.set(\cutoff, ~val_F.linexp(0,1000, 80,4000), \volume, 1);
		});
		0.01.wait;
	}.loop;
}).play;
)

~val_G;

//register Synth with volume 0
~synth_G = Synth(\circle, [\in_num,~val_G.linexp(0,1000, 30,18000), \volume,0]);
(
~control_G = Routine.new({
	{
		if(~val_G.isNegative,{
			~synth_G.set(\volume, 0);
		},
			{
			~synth_G.set(\in_num, ~val_G.linexp(0,1000, 30,18000), \volume, 1);
		});
		0.01.wait;
	}.loop;
}).play;
)

~val_H;

//register Synth with volume 0
~synth_H = Synth(\mixed, [\in_num,~val_H.linexp(0,1000, 0.001,1), \volume,0]);
(
~control_H = Routine.new({
	{
		if(~val_H.isNegative,
		{
				~synth_H.set(\volume, 0);

			},
			{
				~synth_H.set(\in_num, ~val_H.linexp(0,1000, 0.001,1), \volume, 1);
		});
		0.01.wait;
	}.loop;
}).play;
)



~control_F.stop;
~synth_F.free;

~control_G.stop;
~synth_G.free;

~control_H.stop;
~synth_H.free;



s.freeAll;

~port.close;

s.quit;
