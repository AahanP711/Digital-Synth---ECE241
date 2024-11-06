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
