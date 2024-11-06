
/*Try just drawing one pixel on the screen. This will use a much simpler FSM, so it
will be easier to get working. In simulation, you just want to see that the correct
inputs make it to the VGA adapter interface. Make the first pixel red. This requires
you to set X, set Y, and iColour. Then set iPlotBox to plot the pixel. Draw several
pixels at different coordinates. Change the colour to green, set iPlotBox, then blue,
set iPlotBox and see if the pixel colour changes accordingly each time. This will test
that you can input X and Y correctly and input the correct colour. If you can do this,
then you know that you are using the X, Y and iColour inputs correctly.*/

module part2(iResetn,iPlotBox,iBlack,iColour,iLoadX,iXY_Coord,iClock,oX,oY,oColour,oPlot,oDone);
  parameter X_SCREEN_PIXELS = 8'd160;
  parameter Y_SCREEN_PIXELS = 7'd120;

  input wire iResetn, iPlotBox, iBlack, iLoadX;
  input wire [2:0] iColour;
  input wire [6:0] iXY_Coord;
  input wire 	    iClock;
  output wire [7:0] oX;         // VGA pixel coordinates
  output wire [6:0] oY;

  output wire [2:0] oColour;     // VGA pixel colour (0-7)
  output wire 	     oPlot;       // Pixel draw enable
  output wire       oDone;       // goes high when finished drawing frame

  // Intermediate signals
  wire oPlotDatapath, oDoneControlFSM;  

  // Instantiate Datapath module
  Datapath datapath_inst (
    .iClock(iClock),
    .iLoadX(iLoadX),
    .iPlotBox(iPlotBox),
    .iXY_Coord(iXY_Coord),
    .oX(oX),
    .oY(oY),
    .oColour(oColour),
    .oPlot(oPlotDatapath)
  );

  // Instantiate ControlFSM module
  ControlFSM controlFSM_inst (
    .iClock(iClock),
    .iResetn(iResetn),
    .iPlotBox(iPlotBox),
    .oPlot(oPlot),
    .oDone(oDoneControlFSM)
  );

  // Connect intermediate signals to outputs
  assign oPlot = oPlotDatapath;
  assign oDone = oDoneControlFSM;

endmodule // part2

module Datapath(
  input wire iClock,       // Clock input
  input wire iLoadX,       // Load X coordinate control
  input wire iPlotBox,     // Plot pixel control
  input wire [6:0] iXY_Coord,  // Input coordinate
  output wire [7:0] oX,     // Output X coordinate
  output wire [6:0] oY,     // Output Y coordinate
  output wire [2:0] oColour, // Output pixel color
  output wire oPlot         // Output plot control
);

  reg [7:0] X_reg;  // Register to store X coordinate
  reg [6:0] Y_reg;  // Register to store Y coordinate

  // Center coordinates
  localparam X_CENTER = 160 / 2;
  localparam Y_CENTER = 120 / 2;

  // Default values
  assign oX = X_reg;
  assign oY = Y_reg;
  assign oColour = 3'b100;  // Red color

  always @(posedge iClock) begin
    if (iLoadX) begin
      // Load X when iLoadX is high
      X_reg <= iXY_Coord;
    end else if (iPlotBox) begin
      // Load Y and color when iPlotBox is high
      Y_reg <= iXY_Coord;
      oPlot <= 1'b1;
    end
  end
endmodule

module ControlFSM(
  input wire iClock,    // Clock input
  input wire iResetn,   // Active low reset
  input wire iPlotBox,  // Plot pixel control
  output wire oPlot,    // Output plot control
  output wire oDone     // Output done control
);

  reg [2:0] currentState;  // FSM state register

  // FSM states
  localparam IDLE = 3'b000;

  always @(posedge iClock or negedge iResetn) begin
    if (~iResetn) begin
      // Reset FSM to IDLE state
      currentState <= IDLE;
      oPlot <= 1'b0;
      oDone <= 1'b1;
    end else begin
      case (currentState)
        IDLE: begin
          if (iPlotBox) begin
            // Transition to IDLE state and set plot control
            currentState <= IDLE;
            oPlot <= 1'b1;
            oDone <= 1'b1;
          end
        end
      endcase
    end
  end
endmodule

