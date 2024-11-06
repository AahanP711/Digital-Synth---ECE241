`timescale 1ns / 1ps

// image generator of a road and a sky 640x480 @ 60 fps

////////////////////////////////////////////////////////////////////////
module VGA(
	input clk,           // 50 MHz
	input [17:0] note,
	output o_hsync,      // horizontal sync
	output o_vsync,	     // vertical sync
	output [3:0] o_red,
	output [3:0] o_blue,
	output [3:0] o_green  
);

	reg [9:0] counter_x = 0;  // horizontal counter
	reg [9:0] counter_y = 0;  // vertical counter
	reg [3:0] r_red = 0;
	reg [3:0] r_blue = 0;
	reg [3:0] r_green = 0;
	
	reg reset = 0;  // for PLL
	
	wire clk25MHz;
	parameter N0x = 0;
	parameter N0y = 0;

	parameter N3x = 0;
	parameter N3y = 0;

	parameter N5x = 0;
	parameter N5y = 0;

	parameter N6x = 0;
	parameter N6y = 0;

	parameter N7x = 0;
	parameter N7y = 0;

	parameter N10x = 0;
	parameter N10y = 0;

	parameter N12x = 0;
	parameter N12y = 0;

	parameter N15x = 0;
	parameter N15y = 0;

	parameter N17x = 0;
	parameter N17y = 0;
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// clk divider 50 MHz to 25 MHz
	ip ip1(
		.areset(reset),
		.inclk0(clk),
		.c0(clk25MHz),
		.locked()
		);  
	// end clk divider 50 MHz to 25 MHz

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// counter and sync generation
	always @(posedge clk25MHz)  // horizontal counter
		begin 
			if (counter_x < 799)
				counter_x <= counter_x + 1;  // horizontal counter (including off-screen horizontal 160 pixels) total of 800 pixels 
			else
				counter_x <= 0;              
		end  // always 
	
	always @ (posedge clk25MHz)  // vertical counter
		begin 
			if (counter_x == 799)  // only counts up 1 count after horizontal finishes 800 counts
				begin
					if (counter_y < 525)  // vertical counter (including off-screen vertical 45 pixels) total of 525 pixels
						counter_y <= counter_y + 1;
					else
						counter_y <= 0;              
				end  // if (counter_x...
		end  // always
	// end counter and sync generation  

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// hsync and vsync output assignments
	assign o_hsync = (counter_x < 96) ? 1:0;  // hsync high for 96 counts                                                 
	assign o_vsync = (counter_y < 2) ? 1:0;   // vsync high for 2 counts
	// end hsync and vsync output assignments

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// pattern generate
		always @ (posedge clk)
		begin
			if(note[0] == 1)begin
				if(counter_y > N0y - 10 && counter_y < N0y + 10)begin
					if(counter_x > N0x-10 && counter_x < N0x+10)begin
						r_red = 4'hF;
						r_blue = 4'h0;
						r_green = 4'h0;
					end
				end
			end
			
			else if(note[3] == 1)begin
				if(counter_y > N3y - 10 && counter_y < N3y + 10)begin
					if(counter_x > N3x-10 && counter_x < N3x+10)begin
						r_red = 4'hF;
						r_blue = 4'h0;
						r_green = 4'h0;
					end
				end
			end

			else if(note[5] == 1)begin
				if(counter_y > N5y - 10 && counter_y < N5y + 10)begin
					if(counter_x > N5x-10 && counter_x < N5x+10)begin
						r_red = 4'hF;
						r_blue = 4'h0;
						r_green = 4'h0;
					end
				end
			end

			else if(note[6] == 1)begin
				if(counter_y > N6y - 10 && counter_y < N6y + 10)begin
					if(counter_x > N6x-10 && counter_x < N6x+10)begin
						r_red = 4'hF;
						r_blue = 4'h0;
						r_green = 4'h0;
					end
				end
			end

			else if(note[7] == 1)begin
				if(counter_y > N7y - 10 && counter_y < N7y + 10)begin
					if(counter_x > N7x-10 && counter_x < N7x+10)begin
						r_red = 4'hF;
						r_blue = 4'h0;
						r_green = 4'h0;
					end
				end
			end

			else if(note[10] == 1)begin
				if(counter_y > N10y - 10 && counter_y < N10y + 10)begin
					if(counter_x > N10x-10 && counter_x < N10x+10)begin
						r_red = 4'hF;
						r_blue = 4'h0;
						r_green = 4'h0;
					end
				end
			end

			else if(note[12] == 1)begin
				if(counter_y > N12y - 10 && counter_y < N12y + 10)begin
					if(counter_x > N12x-10 && counter_x < N12x+10)begin
						r_red = 4'hF;
						r_blue = 4'h0;
						r_green = 4'h0;
					end
				end
			end

			else if(note[15] == 1)begin
				if(counter_y > N15y - 10 && counter_y < N15y + 10)begin
					if(counter_x > N15x-10 && counter_x < N15x+10)begin
						r_red = 4'hF;
						r_blue = 4'h0;
						r_green = 4'h0;
					end
				end
			end

			else if(note[17] == 1)begin
				if(counter_y > N17y - 10 && counter_y < N17y + 10)begin
					if(counter_x > N17x-10 && counter_x < N17x+10)begin
						r_red = 4'hF;
						r_blue = 4'h0;
						r_green = 4'h0;
					end
				end
			end
		end
						
	// end pattern generate

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// color output assignments
	// only output the colors if the counters are within the adressable video time constraints
	assign o_red = (counter_x > 144 && counter_x <= 783 && counter_y > 35 && counter_y <= 514) ? r_red : 4'h0;
	assign o_blue = (counter_x > 144 && counter_x <= 783 && counter_y > 35 && counter_y <= 514) ? r_blue : 4'h0;
	assign o_green = (counter_x > 144 && counter_x <= 783 && counter_y > 35 && counter_y <= 514) ? r_green : 4'h0;
	// end color output assignments
	
endmodule  // VGA_image_gen
