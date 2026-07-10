module differentiator #(
    parameter    DWIDTH = 12
) (
    input                     clock,
    input                     reset,

    input                     ivalid,
    input  [DWIDTH - 1 : 0]   idata,

    output                    ovalid,
    output [DWIDTH - 1 : 0]   odata
);

reg              ovalid_reg;
reg [DWIDTH-1:0] odata_reg;
reg [DWIDTH-1:0] odata_buf;
always @(posedge clock or posedge reset) begin
    if(reset) begin
        odata_reg  <= 0;
        odata_buf  <= 0;
        ovalid_reg <= 0;
    end
    else begin
        if (ivalid) begin
            ovalid_reg <= 1'b1;
            odata_reg <= idata;
            odata_buf <= $signed(idata) - $signed(odata_reg);
        end
        else begin
            ovalid_reg <= 1'b0;
            odata_buf <= odata_buf; // 保持上一次的值
        end
    end
end

assign odata = odata_buf;
assign ovalid = ovalid_reg;

endmodule  //Differentiator
