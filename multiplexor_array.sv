module multiplexor_array #(
    parameter DATA_WIDTH = 16
)(
    input  logic                  clk,
    input  logic                  rst_n,
    // DMA inputs
    input  logic [DATA_WIDTH-1:0] dma_data,
    input  logic                  dma_valid,
    // Feedback path from activation unit
    input  logic [DATA_WIDTH-1:0] feedback_data,
    input  logic                  feedback_valid,
    // Control
    input  logic [1:0]            sel,  // Selects the data source
    // Output to vector register file
    output logic [DATA_WIDTH-1:0] out_data,
    output logic                  out_valid,
    input  logic                  out_ready
);

    // Internal signals for output
    logic [DATA_WIDTH-1:0] selected_data;
    logic                  selected_valid;
    
    // Multiplexer logic
    always_comb begin
        case (sel)
            2'b00: begin  // Idle state
                selected_data = '0;
                selected_valid = 1'b0;
            end
            
            2'b01: begin  // Select DMA data
                selected_data = dma_data;
                selected_valid = dma_valid;
            end
            
            2'b10, 2'b11: begin  // Select feedback data
                selected_data = feedback_data;
                selected_valid = feedback_valid;
            end
            
            default: begin
                selected_data = '0;
                selected_valid = 1'b0;
            end
        endcase
    end
    
    // Output assignment
    assign out_data = selected_data;
    assign out_valid = selected_valid;

endmodule
