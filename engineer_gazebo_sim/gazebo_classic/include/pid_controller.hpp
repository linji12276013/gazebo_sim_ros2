#pragma once

template<typename T>
T clamp(T value, T low, T high) {
    return (value < low) ? low : (high < value) ? high : value;
}

struct PidController
{
    // 参数
    double pid[3]{0.0};

    // 输出限制
    double output_min{-1e9};
    double output_max{1e9};

    // 内部状态
    double prev_error{0.0};
    double integral{0.0};
    bool initialized{false};

    double setpoint{0.0};
    double measured_value{0.0};

    // 设置增益
    void setGains(double p, double i, double d)
    {
        pid[0] = p; pid[1] = i; pid[2] = d;
    }

    // 设置输出限制
    void setOutputLimits(double min_val, double max_val)
    {
        output_min = min_val; output_max = max_val;
    }

    // 重置
    void reset()
    {
        prev_error = 0.0;
        integral = 0.0;
        initialized = false;
    }

    // 计算控制量
    double compute(double dt)
    {
        double error = setpoint - measured_value;
        integral += error * dt;

        double derivative = 0.0;
        if (initialized)
            derivative = (error - prev_error) / dt;
        else
            initialized = true;

        double output = pid[0] * error + pid[1] * integral + pid[2] * derivative;
        output = clamp(output, output_min, output_max);

        prev_error = error;
        return output;
    }
};
