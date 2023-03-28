module ultrasonic(trigger, echo, reset, clk, test);
    
    input clk, reset, echo;
    output trigger, test;
    reg [15:0] pulse;

    localparam trigger_count = 9;
    localparam stall_count = 100;

    reg[15:0] counter;
    reg [2:0] state;

    // States:
    localparam trigger_state = 0;
    localparam wait_for_pulse_state = 1;
    localparam pulse_count_state = 2;
    localparam latch_pulse_state = 3;
    localparam wait_state = 4;


    initial begin
        state <= trigger_state;
        counter <= 0;
        pulse <= 0;
    end


    assign trigger = state == trigger_state;
    assign test = pulse < (16'd15);

   

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
                state <= wait_state;
                pulse <= counter;
                counter <= 0;
            end

            wait_state: begin
                if (counter < 100) begin
                    counter <= counter + 1;
                end
                else begin 
                    state <= trigger_state;
                    counter <= 0;
                end
            end


        endcase
    end




endmodule
