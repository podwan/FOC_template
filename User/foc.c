#include "foc.h"
#include "bldcMotor.h"
#include "math_utils.h"
#include "comm.h"

DQVoltage_s voltage;
DQCurrent_s current;

/******************************************************************************/

void clarke(float iA, float iB, float iC, float *iAlpha, float *iBeta)
{
    *iAlpha = (2 / 3) * (iA - iB / 2 - iC / 2);
    *iBeta = (1 / SQRT(3)) * (iB - iC);
}

void park(float iAlpha, float iBeta, float theta, float *iD, float *iQ)
{
    *iD = iAlpha * CosApprox(theta) + iBeta * SinApprox(theta);
    *iQ = -iAlpha * SinApprox(theta) + iBeta * CosApprox(theta);
}

void revParkOperate(float uD, float uQ, float theta, float *uAlpha, float *uBeta)
{
    *uAlpha = uD * CosApprox(theta) - uQ * SinApprox(theta);
    *uBeta = uD * SinApprox(theta) + uQ * CosApprox(theta);
}

#if 1
void setPhaseVoltage(float _voltageQ, float _voltageD, float _angleElectrical)
{
    float uOut;

    if (_voltageD != 0)
    {
        uOut = SQRT(_voltageD * _voltageD + _voltageQ * _voltageQ) / voltagePowerSupply;
        _angleElectrical = normalizeAngle(_angleElectrical + atan2(_voltageQ, _voltageD));
    }
    else
    {
        uOut = _voltageQ / voltagePowerSupply;
        _angleElectrical = normalizeAngle(_angleElectrical + _PI_2);
    }

    uint8_t sec = (int)(floor(_angleElectrical / _PI_3)) + 1;
    float t1 = _SQRT3 * SinApprox((float)(sec)*_PI_3 - _angleElectrical) * uOut;
    float t2 = _SQRT3 * SinApprox(_angleElectrical - ((float)(sec)-1.0f) * _PI_3) * uOut;
    float t0 = 1 - t1 - t2;
    float tA, tB, tC;
    switch (sec)
    {
    case 1:
        tA = t1 + t2 + t0 / 2;
        tB = t2 + t0 / 2;
        tC = t0 / 2;
        break;
    case 2:
        tA = t1 + t0 / 2;
        tB = t1 + t2 + t0 / 2;
        tC = t0 / 2;
        break;
    case 3:
        tA = t0 / 2;
        tB = t1 + t2 + t0 / 2;
        tC = t2 + t0 / 2;
        break;
    case 4:
        tA = t0 / 2;
        tB = t1 + t0 / 2;
        tC = t1 + t2 + t0 / 2;
        break;
    case 5:
        tA = t2 + t0 / 2;
        tB = t0 / 2;
        tC = t1 + t2 + t0 / 2;
        break;
    case 6:
        tA = t1 + t2 + t0 / 2;
        tB = t0 / 2;
        tC = t1 + t0 / 2;
        break;
    default:
        tA = 0;
        tB = 0;
        tC = 0;
    }

    // calculate the phase voltages and center
    float voltageA, voltageB, voltageC;
    voltageA = tA * voltagePowerSupply;
    voltageB = tB * voltagePowerSupply;
    voltageC = tC * voltagePowerSupply;
    voltageA = CONSTRAINT(voltageA, 0, voltagePowerSupply);
    voltageB = CONSTRAINT(voltageB, 0, voltagePowerSupply);
    voltageC = CONSTRAINT(voltageC, 0, voltagePowerSupply);
    float dutyA, dutyB, dutyC;
    dutyA = voltageA / voltagePowerSupply;
    dutyB = voltageB / voltagePowerSupply;
    dutyC = voltageC / voltagePowerSupply;
    PWM_GENERATE(dutyA * PWM_PERIOD, dutyB * PWM_PERIOD, dutyC * PWM_PERIOD);

#if SEND_RCC_DATA
    temp[2] = dutyA * PWM_PERIOD;
    temp[3] = dutyB * PWM_PERIOD;
    temp[4] = dutyC * PWM_PERIOD;
    memcpy(tempData, (uint8_t *)&temp, sizeof(temp));
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tempData, 6 * 4);
#endif
}
#else

const char sectorRemap[] = {0, 2, 6, 1, 4, 3, 5};

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    Uq = CONSTRAINT(Uq, 0, uQ_MAX);

    angle_el = normalizeAngle(angle_el);
    float uAlpha, uBeta;

    // rev park
    *uAlpha = uD * CosApprox(theta) - uQ * SinApprox(theta);
    *uBeta = uD * SinApprox(theta) + uQ * CosApprox(theta);
    // SVPWM

    uchar sector;
    sector = (angle_el / _PI_3) + 1;

    float X = _SQRT3 * PWM_PERIOD / U_DC * uBeta;
    float Y = _SQRT3 * PWM_PERIOD / U_DC * (_SQRT3 * uAlpha / 2.0f + uBeta / 2.0f);
    float Z = _SQRT3 * PWM_PERIOD / U_DC * (-_SQRT3 * uAlpha / 2.0f + uBeta / 2.0f);
    float tFirst = 0, tSecond = 0;
    switch (sector)
    {
    case 1:
        tFirst = -Z;
        tSecond = X;
        break;
    case 2:
        tFirst = Z;
        tSecond = Y;
        break;
    case 3:
        tFirst = X;
        tSecond = -Y;
        break;
    case 4:
        tFirst = -X;
        tSecond = Z;
        break;

    case 5:
        tFirst = -Y;
        tSecond = -Z;
        break;

    case 6:
        tFirst = Y;
        tSecond = -X;
        break;
    }

    float t = tFirst + tSecond;

    if (t > PWM_PERIOD)
    {
        tFirst = tFirst / t * PWM_PERIOD;
        tSecond = tSecond / t * PWM_PERIOD;
    }

    int v1 = (PWM_PERIOD - tFirst - tSecond) / 2.0f;
    int v2 = v1 + tFirst;
    int v3 = v2 + tSecond;

    int pwm1Duty, pwm2Duty, pwm3Duty;

    switch (sector)
    {
    case 1:
        pwm1Duty = v1;
        pwm2Duty = v2;
        pwm3Duty = v3;
        break;

    case 2:
        pwm1Duty = v2;
        pwm2Duty = v1;
        pwm3Duty = v3;
        break;

    case 3:
        pwm1Duty = v3;
        pwm2Duty = v1;
        pwm3Duty = v2;
        break;

    case 4:
        pwm1Duty = v3;
        pwm2Duty = v2;
        pwm3Duty = v1;
        break;

    case 5:
        pwm1Duty = v2;
        pwm2Duty = v3;
        pwm3Duty = v1;
        break;

    case 6:
        pwm1Duty = v1;
        pwm2Duty = v3;
        pwm3Duty = v2;
        break;
    }
#if SEND_RCC_DATA
    temp[2] = pwm1Duty;
    temp[3] = pwm2Duty;
    temp[4] = pwm3Duty;
    memcpy(tempData, (uint8_t *)&temp, sizeof(temp));
    HAL_UART_Transmit_D MA(&huart3, (uint8_t *)tempData, 6 * 4);
#endif

    PWM_GENERATE(pwm1Duty, pwm2Duty, pwm3Duty);
}

#endif
