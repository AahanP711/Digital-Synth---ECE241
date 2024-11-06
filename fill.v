// Part 2 skeleton

module fill
	(
		CLOCK_50,						//	On Board 50 MHz
		SW, 							// Your inputs and outputs here
		KEY,							// On Board Keys
		
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]

		//PS2 inout
		PS2_CLK,
		PS2_DAT
	);
	
	// Declare your inputs and outputs here
	
	input			CLOCK_50;				//	50 MHz
	//input	[3:0]	KEY;
				
	
	//input [9:0] SW;	

	inout PS2_CLK;
	inout PS2_DAT;

	// Do not change the following outputs
		output			VGA_CLK;   				//	VGA Clock
		output			VGA_HS;					//	VGA H_SYNC
		output			VGA_VS;					//	VGA V_SYNC
		output			VGA_BLANK_N;				//	VGA BLANK
		output			VGA_SYNC_N;				//	VGA SYNC
		output	[7:0]	VGA_R;   				//	VGA Red[7:0] Changed from 10 to 8-bit DAC
		output	[7:0]	VGA_G;	 				//	VGA Green[7:0]
		output	[7:0]	VGA_B;   				//	VGA Blue[7:0]
	
	wire resetn;
	assign resetn = KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.

	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	wire [17:0] keyBus;
	wire [8:0] Xcord;
	wire [7:0] Ycord;
	wire load, plot;
	wire [9:0] XY;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.

	vga_adapter VGA( 
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			
			/* Signals for the DAC to drive the monitor. */
				.VGA_R(VGA_R),
				.VGA_G(VGA_G),
				.VGA_B(VGA_B),
				.VGA_HS(VGA_HS),
				.VGA_VS(VGA_VS),
				.VGA_BLANK(VGA_BLANK_N),
				.VGA_SYNC(VGA_SYNC_N),
				.VGA_CLK(VGA_CLK)
				);

				defparam VGA.RESOLUTION = "320x240";
				defparam VGA.MONOCHROME = "FALSE";
				defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
				defparam VGA.BACKGROUND_IMAGE = "black_fretboard.mif";
	
	PS2_Demo Guitar(// Inputs
		.clk(CLOCK_50),
		.key(~KEY[3:0]),

		// Bidirectionals
		.PS2_CLK1(PS2_CLK),
		.PS2_DAT1(PS2_DAT),
		
		// Outputs
		.ledr(),
		.keyBusOut(keyBus)
	);
	
	plot P(.clk(CLOCK_50), .Note(keyBus), .locX(Xcord), .locY(Ycord));

	controller C(.clock(CLOCK_50), .resetn(resetn), .Xcord(Xcord), .Ycord(Ycord), .plotbox(plot), .load(load), .XY(XY)); 
	
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.

	part2 P2 (
		.iResetn(resetn),
		.iPlotBox(plot),
		.iBlack(1'b0),
		.iColour(3'b100),
		.iLoadX(load),
		.iXY_Coord(XY),
		.iClock(CLOCK_50),
		.oX(x),
		.oY(y),
		.oColour(colour),
		.oPlot(writeEn));

endmodule

module plot(
    input wire clk,
    input wire [17:0] Note,  // Input from the PS2 decoder
    output reg [8:0] locX,
    output reg [7:0] locY
); 
   //Note parameters 
      parameter N0x = 9'b0; //D3
      parameter N0y = 8'b0; 

      parameter N3x = 9'b100000011; //F3
      parameter N3y = 8'b01111010;

      parameter N5x = 9'b0; //G3
      parameter N5y = 8'b0;

      parameter N6x = 9'b0; //Gs3
      parameter N6y = 8'b0;

      parameter N7x = 9'b0; //F2
      parameter N7y = 8'b0;

      parameter N10x = 9'b0; //G2
      parameter N10y = 8'b0;

      parameter N12x = 9'b0; //As2
      parameter N12y = 8'b0;

      parameter N15x = 9'b0; 
      parameter N15y = 8'b0;

      parameter N17x = 9'b0;
      parameter N17y = 8'b0;

   // Define the output register
   reg [8:0] plotX;
   reg [7:0] plotY;

   // Assign X and Y coordinates based on keyReg
   always@*
   begin
      if(Note[0] == 1)begin
				plotX <= N0x;
                plotY <= N0y;
			end
			
			else if(Note[3] == 1)begin
				plotX <= N3x;
                plotY <= N3y;
			end

			else if(Note[5] == 1)begin
				plotX <= N5x;
                plotY <= N5y;
			end

			else if(Note[6] == 1)begin
				plotX <= N6x;
                plotY <= N6y;
			end

			else if(Note[7] == 1)begin
				plotX <= N7x;
                plotY <= N7y;
			end

			else if(Note[10] == 1)begin
				plotX <= N10x;
                plotY <= N10y;
			end

			else if(Note[12] == 1)begin
				plotX <= N12x;
                plotY <= N12y;
			end

			else if(Note[15] == 1)begin
				plotX <= N15x;
                plotY <= N15y;
			end

			else if(Note[17] == 1)begin
				plotX <= N17x;
                plotY <= N17y;
         end
      locX = plotX;
      locY = plotY;
   end
endmodule

module controller(
	input wire clock,
	input wire resetn,
	input wire [8:0] Xcord,
	input wire [7:0] Ycord,

	output reg plotbox,
	output reg load,
	output reg [9:0] XY
	);

	// Define states
	parameter IDLE = 3'b000;
	parameter SET_XY_X = 3'b001;
	parameter TRIGGER_LOAD = 3'b010;
	parameter SET_XY_Y = 3'b011;
	parameter TRIGGER_PLOTBOX = 3'b100;

	// Define state register
	reg [2:0] state;

	reg [4:0] counter1, counter2;

	// State transition and logic
	always @(posedge clock)
	begin
	
		// Default values
		if (~resetn)
		begin
			state <= IDLE;
			plotbox <= 0;
			load <= 0;
			XY <= 10'b0;
		end
		else
		case (state)

			IDLE: begin
				counter1 <= 5'b0;
				counter2 <= 5'b0;
				if (Xcord != 10'b0 && Ycord != 10'b0)
					state <= #1 SET_XY_X;
			end

			SET_XY_X: begin
				XY <= Xcord;
				state <= #1 TRIGGER_LOAD;
			end

			TRIGGER_LOAD: begin
				// You may need to adjust the number of clock cycles based on your requirements
				// For example, if your clock frequency is 50 MHz and you want to set load for 10 cycles,
				// you might use a counter and transition to the next state after 10 cycles.
				
				load <= 1;
				if(counter1 == 5'b0100)begin
					load <=0;
					state <= #1 SET_XY_Y;
				end
				counter1 = counter1 + 1;				
			end

			SET_XY_Y: begin
				XY <= Ycord;
				state <= #1 TRIGGER_PLOTBOX;
			end

			TRIGGER_PLOTBOX: begin
				plotbox <= 1;
				if(counter2 == 5'b0100)begin
					plotbox <= 0;
					state <= (Xcord == 10'b0 && Ycord == 10'b0) ? IDLE : TRIGGER_PLOTBOX; // Similar to TRIGGER_LOAD, use a counter to control the duration of the plotbox signal
				end
				counter2 = counter2 + 1;
						
					end
		endcase	
	end

	// Reset output values

endmodule
