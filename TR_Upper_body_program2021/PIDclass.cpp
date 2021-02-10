#include "PIDclass.h"

// 繧ｳ繝ｳ繧ｹ繝医Λ繧ｯ繧ｿPID繝代Λ繝｡繝ｼ繧ｿ繧貞ｼ墓焚繧堤畑縺�縺ｦ蛻晄悄蛹悶☆繧�
PID::PID(float xKp, float xKi, float xKd, float xint_time)
{
    Kp = xKp;
    Ki = xKi;
    Kd = xKd;
    int_time = xint_time;

    preError = 0.0; // 1蛟句燕縺ｮ繧ｨ繝ｩ繝ｼ縺ｮ蛟､
    intError = 0.0; // 遨榊�蛟､縺ｮ蛻晄悄蛹�

    init_done = false;
}

float PID::PIDinit(float ref, float act)
{
    preError = ref - act;
    intError = 0.0; // 遨榊�蛟､縺ｮ蛻晄悄蛹�

    init_done = true;
}

// PID蛻ｶ蠕｡縺ｮ螳滉ｽ馴Κ
float PID::getCmd(float ref, float act, float maxcmd)//逶ｮ讓吶��迴ｾ蝨ｨ縺ｮ騾溷ｺｦ縲�荳企剞
{
    float cmd, Error, dError;
    cmd = 0.0;

    if(init_done) {
        Error = ref - act;
        cmd += Error * Kp;

        dError = (Error - preError)/int_time;// / int_time; int_time縺�0.01縺ｮ縺ｨ縺硬Error縺ｮ蛟､縺悟､ｧ縺阪￥縺ｪ繧翫☆縺弱※縺励∪縺�縺ｮ縺ｧ繧ｳ繝｡繝ｳ繝医い繧ｦ繝�
        cmd += dError * Kd;

        intError += (Error + preError) / 2 * int_time;
        cmd += intError * Ki;

        preError = Error;

        if(cmd > maxcmd) cmd = maxcmd;
        if(cmd < -maxcmd) cmd = -maxcmd;
    }
    return cmd;
}

void PID::setPara(float xKp, float xKi, float xKd)
{
    Kp = xKp;
    Ki = xKi;
    Kd = xKd;
}

