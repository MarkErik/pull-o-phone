SerialPort.devices;

~port = SerialPort.new("/dev/tty.usbmodem14101", 115200);

s.boot;


/*

Mapping of ascii letters and values from Serial port to Synths
(same between Arduino and SuperCollider)

e - small square box
f - large square box
g - medium cylinder
h - sandpaper small square box

e - small box - resistor value of 2.36K (Red Dot) will give analog Pin reading of: 188
f - large square box - resistor value of 6.19K (Blue Dot) will give analog Pin reading of:  378
g - medium cylinder - resistor value of 14.6K (Green Dot) will give analog Pin reading of: ~601-602
h - sandpaper small square box - resistor value of 35K (Purple Dot) will give analog Pin reading of: 801

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

m = Synth.new(\smallSquare);

(
SynthDef.new(\bigSquare, {
	arg cutoff=600, volume=1;
	var sig;
	sig = Saw.ar([50,51]);
	sig = RLPF.ar(sig, cutoff.lag(0.02), 0.25,0.3);
	Out.ar(0,sig*volume*0.9);
}).add;
)

n = Synth.new(\bigSquare);
n.set(\cutoff, 1600);

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

c = Synth.new(\circle);

(
SynthDef.new(\sandy, {
	arg freq = 3000, volume = 1; //range from 3000 to 18000
	var sig;
	sig = PinkNoise.ar(SinOsc.kr(4));
	sig = LPF.ar(sig, freq);
	Out.ar([0,1],sig*volume*0.8);
}).add;
)

d = Synth.new(\sandy);



//register Synth with volume 0
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


//register Synth with volume 0
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



//register Synth with volume 0
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


//register Synth with volume 0
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


~control_E.stop;
~synth_E.free;

~control_F.stop;
~synth_F.free;

~control_G.stop;
~synth_G.free;

~control_H.stop;
~synth_H.free;



s.freeAll;

~port.close;

s.quit;
