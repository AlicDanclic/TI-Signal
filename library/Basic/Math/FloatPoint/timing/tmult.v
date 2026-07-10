module tmult #(
        parameter EXP = 5,
        parameter FRA = 10
    ) (
        input              aclk,
        input              aresetn,

        // S_AXIS_A输入
        input  [EXP+FRA:0] s_axis_a_tdata,
        input              s_axis_a_tvalid,
        output             s_axis_a_tready,

        // S_AXIS_B输入
        input  [EXP+FRA:0] s_axis_b_tdata,
        input              s_axis_b_tvalid,
        output             s_axis_b_tready,

        // M_AXIS_RESULT输出
        //m_axis_result_tdata: Output specifications, non-specified number output to 0
        output [EXP+FRA:0] m_axis_result_tdata,
        //the output is invalid
        output             m_axis_result_tvalid,
        //input              m_axis_result_tready,

        //Zero: flag[0]=1  Inf: flag[1] = 1  NaN: flag[2] = 1
        output [2:0]       flag
    );

    wire 	        signA, signB;
    wire [EXP-1:0]	expoA, expoB;
    wire [FRA:0]	fracA, fracB;
    wire            M_unpack_A_tvalid;
    wire            M_unpack_B_tvalid;

    // 分解浮点数的各个部分
    // 消耗一个时钟周期
    unpack_sequential #(
        .EXP (EXP),
        .FRA (FRA))
    u_unpack_sequential(
        .aclk           (aclk),
        .aresetn        (aresetn),

        .inA            (s_axis_a_tdata),
        .S_A_tvalid     (s_axis_a_tvalid),

        .inB            (s_axis_b_tdata),
        .S_B_tvalid     (s_axis_b_tvalid),

        .signA          (signA),
        .expoA          (expoA),
        .fracA          (fracA),
        .M_A_tvalid     (M_unpack_A_tvalid),

        .signB          (signB),
        .expoB          (expoB),
        .fracB          (fracB),
        .M_B_tvalid     (M_unpack_B_tvalid)
    );

    reg  [EXP:0]       r_iexpo;
    reg  [EXP-1:0]     iexpo;
    reg  [2*FRA+1:0]   fraction;
    reg                r_sign;
    reg                d_sign;
    reg                sign;
    reg  [FRA:0]       r_ifrac;
    reg                r_S_normal_tvalid;
    reg                S_normal_tvalid;
    reg                abnormal_flag;

    // 尾数相乘，指数相加并对阶处理
    // 消耗两个时钟周期
    always @(posedge aclk or posedge aresetn) begin
        if(aresetn) begin
            r_iexpo             <= 1'b0;
            iexpo               <= 1'b0;
            fraction            <= 1'b0;
            r_sign              <= 1'b0;
            d_sign              <= r_sign;
            sign                <= d_sign;
            r_ifrac             <= 1'b0;
            r_S_normal_tvalid   <= 1'b0;
            S_normal_tvalid     <= r_S_normal_tvalid;
            abnormal_flag       <= 1'b1;
        end
        else if(M_unpack_A_tvalid && M_unpack_B_tvalid) begin
            // 判断输入是否为规格化数
            // 输入：大 输出：大
            if(expoA == 2**EXP-1 || expoB == 2**EXP-1)begin
                sign            <= signA ^ signB;
                iexpo           <= 2**EXP - 2;
                r_ifrac         <= {(FRA+1){1'b0}};
                abnormal_flag   <= 1'b1;
            end
            else if(expoA == {(EXP){1'b0}} || expoB == {(EXP){1'b0}})begin
                sign            <= 1'b0;
                iexpo           <= 2**EXP - 1;
                r_ifrac         <= {(FRA+1){1'b0}};
                abnormal_flag   <= 1'b1;
            end
            else begin
                r_iexpo             <= expoA + expoB;
                fraction            <= fracA * fracB;
                r_sign              <= signA ^ signB;
                r_S_normal_tvalid   <= 1'b1;
                S_normal_tvalid     <= r_S_normal_tvalid;
                // 判断结果是否为规格化数
                if(r_iexpo <= (2**(EXP-1)-1))begin
                    sign                <= 1'b0;
                    iexpo               <= 2**(EXP-1)-1;
                    r_ifrac             <= {(FRA+1){1'b0}};
                    abnormal_flag       <= 1'b1;
                end
                else if(r_iexpo >= 2**(EXP) + 2**(EXP-1) - 1)begin
                    d_sign          <= r_sign;
                    sign            <= d_sign;
                    iexpo           <= 2**EXP - 2;
                    r_ifrac         <= {(FRA+1){1'b0}};
                    abnormal_flag   <= 1'b1;
                end
                else begin
                    iexpo               <= r_iexpo - (2**(EXP-1) - 2);
                    d_sign              <= r_sign;
                    sign                <= d_sign;
                    abnormal_flag       <= 1'b0;
                    // 根据低位对高位进行进位处理
                    if(fraction[FRA])begin
                        r_ifrac <= fraction[2*FRA+1:FRA+1] + 1'b1;
                    end
                    else begin
                        r_ifrac <= fraction[2*FRA+1:FRA+1];
                    end
                end
            end
        end
        else begin
            /*
            r_iexpo             <= 1'b0;
            iexpo               <= iexpo;
            fraction            <= 1'b0;
            r_ifrac             <= r_ifrac;
            r_sign              <= 1'b0;
            d_sign              <= r_sign;
            sign                <= d_sign;
            */
            r_S_normal_tvalid   <= 1'b0;
            S_normal_tvalid     <= r_S_normal_tvalid;
            abnormal_flag       <= abnormal_flag;
        end
    end

    wire [FRA + 1 : 0] ifrac;
    wire [FRA - 1 : 0] ofrac;
    wire [EXP - 1 : 0] oexpo;
    wire               M_normal_tvalid;

    assign ifrac = abnormal_flag ? {1'b1,r_ifrac} : {1'b0,r_ifrac};

    // 指定结果
    // 消耗一个时钟周期
    normal_sequential #(
        .EXP (EXP),
        .FRA (FRA))
    u_normal_sequential(
        .aclk           (aclk),
        .aresetn        (aresetn),

        .iexpo          (iexpo),
        .ifrac          (ifrac),
        .S_tvalid       (S_normal_tvalid),

        .oexpo          (oexpo),
        .ofrac          (ofrac),
        .M_tvalid       (M_normal_tvalid)
    );

    // 判断是否有特殊数
    cksp #(
        .EXP (EXP),
        .FRA (FRA))
    u_cksp(
        .expo	  (oexpo),
        .frac	  (ofrac),

        .flag 	  (flag)
    );

    // 组合浮点数的各个部分
    pack #(
        .EXP 		( EXP 		),
        .FRA 		( FRA 		))
    u_pack(
        // 端口
        .out  		( m_axis_result_tdata),
        .sign 		( sign 		),
        .expo 		( oexpo 	),
        .frac 		( ofrac 	)
    );

assign s_axis_a_tready = s_axis_a_tvalid;
assign s_axis_b_tready = s_axis_b_tvalid;
assign m_axis_result_tvalid = M_normal_tvalid;

endmodule  // 乘法