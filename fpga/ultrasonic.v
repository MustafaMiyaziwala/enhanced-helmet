module ultrasonic #(parameter N = 5) (clk, echo, trigger, pulse);
    
    input clk, echo;
    output trigger;
    output reg [14:0] pulse;

    localparam trigger_count = 9;
    localparam stall_count = 50;
    localparam count_max = ~(15'd0);

    // circular buffer for rolling average
    reg [14:0] counter;
    reg [14:0] buffer [0:N-1];
    reg [log2(N):0] buffer_pos;
    reg [2:0] state;


    // States:
    localparam trigger_state = 0;
    localparam wait_for_pulse_state = 1;
    localparam pulse_count_state = 2;
    localparam latch_pulse_state = 3;
    localparam stall_state = 4;


    initial begin
        state <= trigger_state;
        buffer_pos <= 0;
        pulse <= 0;
        counter <= 0;

        for (int i = 0; i < N; i = i + 1) begin
            buffer[i] <= 0;
        end
    end

    assign trigger = state == trigger_state;

    // State machine transitions
    always @(posedge clk) begin

        case (state)
            trigger_state: begin
                if (counter >= trigger_count) begin
                    state <= wait_for_pulse_state;
                    counter <= 0;
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
                else if (counter == count_max) begin
                    state <= trigger_state;
                    counter <= 0;
                end
                else begin
                    counter <= counter + 1;
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
            
                reg [18:0] sum = counter;
                for (int i = 1; i < N; i = i + 1) begin
                    sum = sum + buffer[(buffer_pos + i) % N];
                end

                pulse <= sum / N;
                buffer[buffer_pos] <= sum / N;
                buffer_pos <= (buffer_pos + 1) % N;
                counter <= 0;
                state <= stall_state;
            end

            stall_state: begin
                if (counter == stall_count) begin
                    state <= trigger_state;
                    counter <= 0;
                end
                else begin
                    counter <= counter + 1;
                end
            end
        endcase
    end

endmodule
