module ultrasonic(pulse, trigger, echo, reset, clk);
    
    input clk, reset;
    output trigger, echo;
    output reg [15:0] pulse;

    localparam trigger_count = 100; // TODO: base on clock

    reg[15:0] counter;
    reg [1:0] state;

    // States:
    localparam trigger_state = 0;
    localparam wait_for_pulse_state = 1;
    localparam pulse_count_state = 2;
    localparam latch_pulse_state = 3;


    initial begin
        state <= trigger_state;
        counter <= 0;
        pulse <= 0;
    end


    assign trigger = state == trigger_state;


    always @(posedge reset) begin
        state <= trigger_state;
        counter <= 0;
    end

    // State machine transitions
    always @(posedge clk) begin
        
        case (state)
            trigger_state: begin
                if (counter >= trigger_count) begin
                    state <= wait_for_pulse_state;
                end
                else begin
                    counter <= counter + 1;
                end
            end

            wait_for_pulse_state: begin
                if (echo) begin
                    state <= pulse_count_state;
                    counter <= 0;
                end
            end

            pulse_count_state: begin
                if (echo) begin
                    counter <= counter + 1;
                end
                else begin
                    state <= latch_pulse_state;
                end
            end

            latch_pulse_state: begin
                state <= trigger;
                pulse <= counter;
                counter <= 0;
            end
        endcase
    end


endmodule