
module ChordMachine2 (
	// Inputs
	CLOCK_50,
	KEY,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	FPGA_I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT, 

	FPGA_I2C_SCLK,
	SW,
	LEDR,
	PS2_CLK,
	PS2_DAT
);
	//Keyboard Instantiation
	PS2_Demo PIANO(	// Inputs
		.clk(CLOCK_50),
		.key(~KEY[3:0]),

		// Bidirectionals
		.PS2_CLK1(PS2_CLK),
		.PS2_DAT1(PS2_DAT),
		
		// Outputs
		.hex0(HEX0),
		.hex1(HEX1),
		.hex2(HEX2),
		.hex3(HEX3),
		.hex4(HEX4),
		.hex5(HEX5),
		.hex6(HEX6),
		.hex7(HEX7), 
		.ledr(),
		.keyBusOut(keyBus)
	);
	
	inout PS2_CLK;
	inout PS2_DAT;
	
	wire [17:0] keyBus; //For triggering sound modules - from keys 
	//on keyboard

	/*****************************************************************************
	 *                           Parameter Declarations                          *
	 *****************************************************************************/
	//parameter ampDiv = 4; could use this if want to implement volume control
	parameter numNotes = 3; //really num notes - 1

	/*****************************************************************************
	 *                             Port Declarations                             *
	 *****************************************************************************/
	// Inputs
	input				CLOCK_50;
	input		[3:0]	KEY;
	input		[9:0]	SW;

	input				AUD_ADCDAT;

	// Bidirectionals
	inout				AUD_BCLK;
	inout				AUD_ADCLRCK;
	inout				AUD_DACLRCK;

	inout				FPGA_I2C_SDAT;

	// Outputs
	output				AUD_XCK;
	output				AUD_DACDAT;

	output				FPGA_I2C_SCLK;

	output 		[9:0] LEDR;
	
	/*****************************************************************************
	 *                 Internal Wires and Registers Declarations                 *
	 *****************************************************************************/
	// Internal Wires
	wire				audio_in_available;
	wire		[31:0]	left_channel_audio_in;
	wire		[31:0]	right_channel_audio_in;
	wire				read_audio_in;

	wire				audio_out_allowed;
	wire		[31:0]	left_channel_audio_out;
	wire		[31:0]	right_channel_audio_out;
	wire				write_audio_out;
	

	wire [20:0] delay_cnt0, delay_cnt1, delay_cnt2, delay_cnt3, delay_cnt4, delay_cnt5, delay_cnt6, delay_cnt7, delay_cnt8; //were reg signed. now in module.
	wire [20:0] delay, delay2, delay3, delay4, delay5, delay6, delay7, delay8;

	parameter clock = 50000000;
	//Note: these parameters are not in Hz. In "delay" units
	parameter C3 = clock/131; //130.8128
	parameter Eb3 = clock/156; //155.5635	
	parameter F3 = clock/175; //174.6141	
	parameter G3 = clock/196; //195.9977	
	
	parameter C4 = clock/262; //261.6256Hz
	parameter Eb4 = clock/311; //311.1270Hz
	
	parameter A4 = clock/440;
	parameter Bb4 = clock/466;
	parameter B4 = clock/494;
	parameter C5 = clock/523;
	parameter Db5 = clock/554;
	parameter D5 = clock/587;
	parameter Eb5 = clock/622;
	parameter E5 = clock/659;
	parameter F5 = clock/698;
	parameter Gb5 = clock/740;
	parameter G5 = clock/784;
	parameter Ab5 = clock/831;
	parameter A5 = clock/880;
	parameter Bb5 = clock/932;
	parameter B5 = clock/988;
	parameter C6 = clock/1047; //1046.502Hz
	parameter Eb6 = clock/1244; //1244.508Hz
	parameter F6 = clock/1397; //1396.913Hz

	wire squareEn = SW[9];
	wire loopEn = SW[5];

	//WAVE GENERATION MODULE INSTANTIATION
	//squareWave(CLOCK_50, snd, delay);
	squareWave s0(CLOCK_50, squareOut0, C3, squareEn); //changed first 4 notes to match blues scale
	squareWave s1(CLOCK_50, squareOut1, Eb3, squareEn);
	squareWave s2(CLOCK_50, squareOut2, F3, squareEn);
	squareWave s3(CLOCK_50, squareOut3, G3, squareEn);
	squareWave s4(CLOCK_50, squareOut4, E5, squareEn);
	squareWave s5(CLOCK_50, squareOut5, F5, squareEn);
	squareWave s6(CLOCK_50, squareOut6, G5, squareEn);
	squareWave s7(CLOCK_50, squareOut7, A5, squareEn);
	squareWave s8(CLOCK_50, squareOut8, B5, squareEn);

	//Out-of Phase sawWave Generators (p is for phase)
	//----------------------------------------------------------------------------------------
	//For keyboard - defining blues scale
	squareWave s0k(CLOCK_50, squareOut0k, C5, squareEn);
	squareWave s1k(CLOCK_50, squareOut1k, Eb5, squareEn);
	squareWave s2k(CLOCK_50, squareOut2k, F5, squareEn);
	squareWave s3k(CLOCK_50, squareOut3k, Gb5, squareEn);
	squareWave s4k(CLOCK_50, squareOut4k, G5, squareEn);
	squareWave s5k(CLOCK_50, squareOut5k, Bb5, squareEn);
	squareWave s6k(CLOCK_50, squareOut6k, C6, squareEn);
	squareWave s7k(CLOCK_50, squareOut7k, Eb6, squareEn); 
	squareWave s8k(CLOCK_50, squareOut8k, F6, squareEn);

	//Output busses
	wire [31:0] o0, o1, o2, o3, o4, o5, o6, o7, o8;
	wire [31:0] o0k, o1k, o2k, o3k, o4k, o5k, o6k, o7k, o8k;
	
	//Wires for DE1 Switch-Activated Chord Modules
	wire [31:0] squareOut0, squareOut1, squareOut2, squareOut3, squareOut4, squareOut5, squareOut6, squareOut7, squareOut8;

	//Wires for the keyboard-activated Chord Modules
	wire [31:0] squareOut0k, squareOut1k, squareOut2k, squareOut3k, squareOut4k, squareOut5k, squareOut6k, squareOut7k, squareOut8k;

	//Assign outputs from wave generators to the finaloutput bus to the audio output.
		assign o0 = (finalOutBus[0] == 0) ? 0 : squareOut0 ; //squareOut0 + sawOut0; //delay_cnt0*88; //88 is maxAmplitude/A4 //sawOut0 | sawOut0p;
		assign o1 = (finalOutBus[1] == 0) ? 0 : squareOut1 ;
		assign o2 = (finalOutBus[2] == 0) ? 0 : squareOut2 ;
		assign o3 = (finalOutBus[3] == 0) ? 0 : squareOut3 ;
		assign o4 = (finalOutBus[4] == 0) ? 0 : squareOut4 ;
		assign o5 = (finalOutBus[5] == 0) ? 0 : squareOut5 ;
		assign o6 = (finalOutBus[6] == 0) ? 0 : squareOut6 ;
		assign o7 = (finalOutBus[7] == 0) ? 0 : squareOut7 ;
		assign o8 = (finalOutBus[8] == 0) ? 0 : squareOut8 ;

	//Outputs for the keyboard -add k to eveything
	//0, 3, 5, 6, 7, 10, 12, 15, 17 are degrees of the blues scale!
		assign o0k = (keyBus[0] == 0) ? 0 : squareOut0k ; //squareOut0 + sawOut0; //delay_cnt0*88; //88 is maxAmplitude/A4 //sawOut0 | sawOut0p;
		assign o1k = (keyBus[3] == 0) ? 0 : squareOut1k ;
		assign o2k = (keyBus[5] == 0) ? 0 : squareOut2k ;
		assign o3k = (keyBus[6] == 0) ? 0 : squareOut3k ;
		assign o4k = (keyBus[7] == 0) ? 0 : squareOut4k ;  
		assign o5k = (keyBus[10] == 0) ? 0 : squareOut5k ;
		assign o6k = (keyBus[12] == 0) ? 0 : squareOut6k ;
		assign o7k = (keyBus[15] == 0) ? 0 : squareOut7k ;
		assign o8k = (keyBus[17] == 0) ? 0 : squareOut8k ;


	assign read_audio_in			= audio_in_available & audio_out_allowed;

	assign left_channel_audio_out	= left_channel_audio_in+o0+o1+o2+o3+o4+o5+o6+o7+o8+o0k+o1k+o2k+o3k+o4k+o5k+o6k+o7k+o8k;
	assign right_channel_audio_out	= right_channel_audio_in+o0+o1+o2+o3+o4+o5+o6+o7+o8+o0k+o1k+o2k+o3k+o4k+o5k+o6k+o7k+o8k;
	assign write_audio_out			= audio_in_available & audio_out_allowed;

	/*****************************************************************************
	 *                              Internal Modules                             *
	 *****************************************************************************/

	Audio_Controller Audio_Controller (
		// Inputs
		.CLOCK_50						(CLOCK_50),
		.reset						(0), //was ~KEY[0]

		.clear_audio_in_memory		(),
		.read_audio_in				(read_audio_in),
		
		.clear_audio_out_memory		(),
		.left_channel_audio_out		(left_channel_audio_out),
		.right_channel_audio_out	(right_channel_audio_out),
		.write_audio_out			(write_audio_out),

		.AUD_ADCDAT					(AUD_ADCDAT),

		// Bidirectionals
		.AUD_BCLK					(AUD_BCLK),
		.AUD_ADCLRCK				(AUD_ADCLRCK),
		.AUD_DACLRCK				(AUD_DACLRCK),


		// Outputs
		.audio_in_available			(audio_in_available),
		.left_channel_audio_in		(left_channel_audio_in),
		.right_channel_audio_in		(right_channel_audio_in),

		.audio_out_allowed			(audio_out_allowed),

		.AUD_XCK					(AUD_XCK),
		.AUD_DACDAT					(AUD_DACDAT)
	);

	avconf #(.USE_MIC_INPUT(1)) avc (
		.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
		.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
		.CLOCK_50					(CLOCK_50),
		.reset						(0) //was ~KEY[0]
	);

		
	//CHORD REGISTER LOGIC
	//---------------------------------------------------------------------------------------
	//Register Code
	wire [9:0] qbus0;
	wire [9:0] qbus1;
	wire [9:0] qbus2;
	wire [9:0] qbus3;
	wire [9:0] qbus0pt; //passthrough, enabled with switches
	wire [9:0] qbus1pt;
	wire [9:0] qbus2pt;
	wire [9:0] qbus3pt;
	wire [9:0] finalOutBus; //10-bit in case we want to use all switches
	//To show switch outputs when in write mode:
	//assign LEDR[numNotes:0] = writeEn ? SW : finalOutBus; //Show note outputs for current chord register on LEDs 
	//This LEDR can become a wire bus to send data to a bunch of note-producing modules.
	
	wire writeEn;
	assign writeEn = (SW[numNotes:0] != 0) & !loopEn; //If @ least 1 switch from keyset selected and loopEn == 0, in WRITE MODE. If NO switches are selected, in PLAY mode.
	//Also have to make sure the looper mode isn't on^
	
	//ASSIGNING INPUT STAGE
	wire [numNotes:0] noteKeys = SW[numNotes:0]; //assigning a certain number of switches to be used as notes to play pitches.
	
	//NOTE: KEY IS INVERTED BEFORE BEING SENT IN.
	chordRegister c3(.sw(noteKeys), .clk(CLOCK_50), .key(~KEY[3]), .writeEn(writeEn), .q(qbus3[9:0]));
	chordRegister c2(.sw(noteKeys), .clk(CLOCK_50), .key(~KEY[2]), .writeEn(writeEn), .q(qbus2[9:0]));
	chordRegister c1(.sw(noteKeys), .clk(CLOCK_50), .key(~KEY[1]), .writeEn(writeEn), .q(qbus1[9:0]));
	chordRegister c0(.sw(noteKeys), .clk(CLOCK_50), .key(~KEY[0]), .writeEn(writeEn), .q(qbus0[9:0]));
	
	//loopbus[3] is the 1st beat of the shift registers. To be read @ rate of 120BPM
	assign qbus3pt = (~KEY[3] | loopbus3[3]) ? qbus3 : 0; //Blend audio from the looper. Can also play chords (in play mode) over the loop
	assign qbus2pt = (~KEY[2] | loopbus2[3]) ? qbus2 : 0; //Have to change to 1 for some reason? Loading into register 2 and 0 are off for some reason?
	assign qbus1pt = (~KEY[1] | loopbus1[3]) ? qbus1 : 0;
	assign qbus0pt = (~KEY[0] | loopbus0[3]) ? qbus0 : 0;

	assign finalOutBus = qbus0pt | qbus1pt | qbus2pt | qbus3pt; //enable simultaneous chord playing. Also to allow hearing chords as you create them in write mode.		

	//Issue:
	//Need to load Q on the negedge of the key
	//When you load a loop register, sometimes the notes are off by 1 clock cycle...
	//ONly problem: led 0 and 2 are reversed in the looper? Chord registers 1 and 3 only load correctly for key 1 or 3. Equivalent for 0 and 2.
	//Potential loading issue.

	//The shift registers store the chords at the correct locations, but they are read at 1-rate-div delay (120bpm) based on
	//when you hit the key to store D.
	

	//LOOPER LOGIC-----------------------------------------------------------------------------------------------------------------------------------
	//Activate looper with SW[5]
	//4 x 4-bit shift registers

	wire [numNotes:0] loopbus0, loopbus1, loopbus2, loopbus3; //each loopbus contains the time-chord activation data for a certain chord register
	wire [numNotes:0] staticloopbus0, staticloopbus1, staticloopbus2, staticloopbus3; 
	//static loopbusses contain the original notes you enter into the sequencer; i.e. not shifting
	wire [26:0] rateOut;
	wire [numNotes:0] LEDPos;
	wire BPMShiftEn;
	
	//Instantiating 1 shift register per chord. (EX: 4 chords, therefore need 4 registers)
	//loopEn is SW[5]. shiftEn is the enable from the clock (rate divider)
	shiftReg4bit chord3(.D(noteKeys), .clk(CLOCK_50), .loopEn(loopEn), .BPMShiftEn(BPMShiftEn), .Q(loopbus3), .key(~KEY[3]), .Qstatic(staticloopbus3), .firstBeatLoad(start), .Dstatic(Dstatic3)); //loopEn is SW[5]
	shiftReg4bit chord2(.D(noteKeys), .clk(CLOCK_50), .loopEn(loopEn), .BPMShiftEn(BPMShiftEn), .Q(loopbus2), .key(~KEY[2]), .Qstatic(staticloopbus2), .firstBeatLoad(start), .Dstatic(Dstatic2));
	shiftReg4bit chord1(.D(noteKeys), .clk(CLOCK_50), .loopEn(loopEn), .BPMShiftEn(BPMShiftEn), .Q(loopbus1), .key(~KEY[1]), .Qstatic(staticloopbus1), .firstBeatLoad(start), .Dstatic(Dstatic1));
	shiftReg4bit chord0(.D(noteKeys), .clk(CLOCK_50), .loopEn(loopEn), .BPMShiftEn(BPMShiftEn), .Q(loopbus0), .key(~KEY[0]), .Qstatic(staticloopbus0), .firstBeatLoad(start), .Dstatic(Dstatic0));
	
	//Used to sync all registers to load on the 1st beat only (therefore, these registers store the value on switches temporarily, until the 
	//1st beat of the bar is reached, which is when a signal is sent on startOfBar to cause the shiftReg4bits modules to load the values of the switches (the values that were last set!).
	//these don't actually act as chord registers. They act as switch (TIME) registers. i.e. store values on switches when key pressed
	wire [3:0] Dstatic3, Dstatic2, Dstatic1, Dstatic0, DstaticLED;
	chordRegister SHIFT3(.sw(noteKeys), .clk(CLOCK_50), .key(~KEY[3]), .writeEn(loopEn), .q(Dstatic3));
	chordRegister SHIFT2(.sw(noteKeys), .clk(CLOCK_50), .key(~KEY[2]), .writeEn(loopEn), .q(Dstatic2));
	chordRegister SHIFT1(.sw(noteKeys), .clk(CLOCK_50), .key(~KEY[1]), .writeEn(loopEn), .q(Dstatic1));
	chordRegister SHIFT0(.sw(noteKeys), .clk(CLOCK_50), .key(~KEY[0]), .writeEn(loopEn), .q(Dstatic0));

	//This shifts an LED to show BPM (shows position - regardless of which register is being written to)
	chordRegister TRACKLED(.sw(9'b000001000), .clk(CLOCK_50), .key(1'b1), .writeEn(loopEn), .q(DstaticLED));
	shiftReg4bit trackLED(.D(4'b1000), .clk(CLOCK_50), .loopEn(loopEn), .BPMShiftEn(BPMShiftEn), .Q(LEDPos), .key(1'b0), .Qstatic(), .firstBeatLoad(start), .Dstatic(DstaticLED)); //THIS IS NOT WORKING!!!!!!!!!!!!!!!!!!!!!

	//TIMING MODULES
	wire [2:0] startOfBar;
	wire start = ~|startOfBar[2:0];	//OUTPUT every 4 bars - sends the signal to start loading at 1st beat of bar		
	RateDivider r1(.q(rateOut), .d(counterMax), .clk(CLOCK_50), .enable(loopEn), .startOfBar(startOfBar)); //Creates the BPM
	
	parameter bpm = 120;
	parameter counterMax = (clock/(bpm/60)) - 1; //Shift out at a rate of 120BPM (120 cycles/min*(1m/60s) = 2Hz) 
	//Therefore, count down from 50M/2 = 24 999 999
	assign BPMShiftEn = ~|rateOut[26:0]; //Send pulse everytime count hits 0
	
	//LED OUTPUT STAGE
	assign LEDR[5] = loopEn ? start : |keyBus;
	assign LEDR[4] = loopEn ? BPMShiftEn : 0; //could just assign BPMshiftEn to an LED, and it would show BPM.
	
	//Want to show what you select per chord register on LEDs, based on which key you hit. 
	//EX: select 1 and 3. hit key 0. 1 and 3 show up, and counter led moves.
	reg [3:0] ledShiftSel; //toggles which chord LED timer counter is being shown. Can toggle
	always @(posedge CLOCK_50) begin
		if (loopEn) begin
			//ledShiftSelect = {~KEY[3], ~KEY[2], ~KEY[1], ~KEY[0]};
			if (~KEY[0]) begin 
				ledShiftSel <= 0;
				ledShiftSel[0] <= 1'b1; //!ledShiftSel[0] //Toggle on/off - really just need to set = 1 here.
			end
			if (~KEY[1]) begin
				ledShiftSel <= 0; //Clear the others
				ledShiftSel[1] <= 1'b1;
			end
			if (~KEY[2]) begin
				ledShiftSel <= 0;
				ledShiftSel[2] <= 1'b1;
			end
			if (~KEY[3]) begin
				ledShiftSel <= 0;
				ledShiftSel[3] <= 1'b1;
			end
			if (!loopEn) begin
				ledShiftSel[3:0] <= 0;
			end
		end
	end
	
		//This is showing the shifted values from loopbuses. ONLY WANT TO SHOW THE SELECTED VALUES. (ie. KEY0 and KEY1 were switches.
	assign LEDR[numNotes:0] = loopEn ? (LEDPos[numNotes:0] | (ledShiftSel[3] ? staticloopbus3 : 0) | (ledShiftSel[2] ? staticloopbus2 : 0) | (ledShiftSel[1] ? staticloopbus1 : 0) | (ledShiftSel[0] ? staticloopbus0 : 0)) : (writeEn ? SW[numNotes:0] : finalOutBus[numNotes:0]); //loopbuses will only be written to when a key is hit (look above)
	
endmodule

//Counts down, outputs on Q to enable only when all digits are 0 (reaches 0).
module RateDivider(q, d, clk, enable, startOfBar); //Not using clear b or enable. - removed parLoad

	output reg [26:0] q;
	output reg [2:0] startOfBar; //3-bit to store 4 beats per bar
	input [26:0] d;
	input clk;
	input enable;
	
	always @(posedge clk)
	begin
		if (startOfBar == 4) begin //Allows all registers to load at the start of a bar only!
			startOfBar <= 0;
		end
		if (q == 0) begin //d is the rate from the mux. The number is loaded when parallel load is high. //HAD ParLoad == 1'b1
			q <= d;
			//This will sync all the shift registers. Will only load D on the 1st beat of every bar (4 beats/bar)
			startOfBar <= startOfBar + 1;
		end
		else if (enable == 1'b1)
			q <= q - 1; //DECREASE counter by 1 only when enable is high
	end

endmodule

//4-Bit Shift Register (to be instantiated 4 times for 4 different chords.)
module shiftReg4bit(D, clk, loopEn, BPMShiftEn, Q, key, Qstatic, firstBeatLoad, Dstatic);
	//Qstatic is the initial load pattern of Q. Does not shift. Used to show on LEDs where you placed a note rhythmically.
	parameter numNotes = 3; //really 4

	input key;
	input [numNotes:0] D; //Only for inst. "loading" into LEDs
	input clk;
	input loopEn;
	input BPMShiftEn; //Pulse received @ rate of 120BPM
	output reg [numNotes:0] Q; //shifting output register	
	output reg [numNotes:0] Qstatic; //Qstatic is really Dstatic. - stores the switch values on key press.
	
	//For managing first beat loading only.
	input firstBeatLoad;
	input [numNotes:0] Dstatic; //Static input from the initial load
	
	always @ (posedge clk) begin	
		if (firstBeatLoad) begin
			Q <= Dstatic; //load the shift register ALWAYS ON THE FIRST BEAT! - could actually load Qstatic here - equivalent //Q <= Qstatic;
		end
		
		//This block is to show loaded switches on LEDs only. (no computation)
		if (loopEn & key) begin //SW[5] must be on, and the key must be pressed to set the chord into position for this chord loop. - HAVING key HERE WILL PREVENT FROM PLAYING DURING LOOP!!!!!!!!!
			//Q <= 4'b000;
			Qstatic <= D; //for showing on LEDs what times were selected.
		end
		
		else if (loopEn & BPMShiftEn) //Load right bit to shift bits left.
		begin 
			Q[3] <= Q[0]; //registers and LEDs: | 3 | 2 | 1 | 0 |
			Q[2] <= Q[3];
			Q[1] <= Q[2];
			Q[0] <= Q[1];	
		end
		
		if (!loopEn) begin
			Q <= 4'b000; //reset when looper mode turned off.
		end
	end

endmodule
	
//CHORD REGISTER MODULE
//Instantiate 4 of these 10-bit note registers to store 4 different chords, which can be selected via a keypress.
//**Want the keypress to activate it's register's outputs (notes) for only as long as the key is pressed.
module chordRegister(sw, clk, key, writeEn, q);

	input [9:0] sw; //Or may have to math numNotes!
	input clk;
	input key;
	input writeEn;
	output reg [9:0] q;
	
	always @ (posedge clk) begin
		if (key == 1'b1) begin
			if (writeEn == 1'b1) begin
				q <= sw;
			end
		end
	end
	
endmodule

//SQUAREWAVE GENERATOR
module squareWave(clk, sndOut, delay, enable);
	//parameter maxAmplitude;
	input enable;
	input clk;
	input [20:0] delay; //21-bit wire because lowest piano pitch is ~27Hz, therefore 50000000/27 < (2^21 = 2 097 152)
	reg signed [20:0] delay_cnt;
	reg snd;
	output reg signed [31:0] sndOut;
	
	always @ (posedge clk) begin
		if(delay_cnt == delay) begin
			delay_cnt <= 0;
			snd <= !snd;
		end else delay_cnt <= delay_cnt + 1;
	end
	
	always @(*) begin
		sndOut <= enable ? (snd ? 32'd20000000 : -32'd20000000) : 0;
	end
endmodule