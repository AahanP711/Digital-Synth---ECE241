module plot(
    input wire clk,
    input wire [17:0] Note,
    output reg [2:0] colour,
    output reg [8:0] x,
    output reg [7:0] y,
    output reg plot
);

//vga_adapter V(.clock(clk), .resetn(1) , .color(color), .x(countX), y(countY), .plot(plot));

// Define the states of the FSM
parameter IDLE = 2'b00;
parameter DECODE = 2'b01;
parameter PLOT = 2'b10;

// Define the state register
reg [1:0] state, next_state;

// Define the output register
reg [8:0] plotX;
reg [7:0] plotY;

reg [8:0] countX;
reg [7:0] countY;

reg reset;


// Define the x and y coordinates of the center of the red square
//parameter CENTER_X = 320;
//parameter CENTER_Y = 240;

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

// Define the FSM logic
always @ (posedge clk) begin
    state <= next_state;
    case (state)
        IDLE: begin
            // Check if a new Note has been received
            
            reset = 1'b1;
            plot = 1'b0;

            if (Note != 18'b0) begin
                next_state = DECODE;
            end else begin
                next_state = IDLE;
            end
        end
       
        DECODE: begin
            // Decode the Note and calculate the x and y coordinates of the red square
            // Set the colour to red
            // Set the plot signal to high
            
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
            
            next_state = PLOT;

        end
        PLOT: begin

            countX = plotX;
            countY = plotY;
            
            if(countY <= plotY + 4 && countX == plotX + 4)begin
                countY = countY + 1;
            end

            if(countX <= plotX + 4) begin
                countX = countX + 1;
            end
            // Plot the red square
            // Reset the plot signald
            if(countX == plotX + 4 && countY == plotY + 4 )begin
                next_state = IDLE;
            end
        end
    endcase
end

// Define the output logic
always @ (*) begin
    case (state)
        IDLE: begin
            // Set the outputs to their default values
            colour = 3'b000;
            x = 9'b0;
            y = 8'b0;
            plot = 1'b0;
            
        end
        DECODE: begin
            colour <= 3'b100;
        end
        PLOT: begin
            plot = 1'b1;
        end
    endcase
end

endmodule


