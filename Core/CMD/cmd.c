#include "cmd.h"
#include "../LED/MAIN_BOARD_RGB/ws2812.h"
#include "../CALIBRATION/IMU/icm42688p_calibration.h"
#include "../ICM42688P/icm42688p.h"
#include "../EEPROM/eeprom.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <main.h>
#include "stm32h7xx_hal.h"


volatile uint8_t cmd_mode = 0; // Command mode flag (0 = normal, 1 = command mode)

// Helper function to parse a float from a string, advancing the pointer
static bool parse_float(const char **str, float *result) {
    char *endptr;
    // Skip leading whitespace
    while (**str == ' ') (*str)++;

    *result = strtof(*str, &endptr);
    if (endptr == *str) {
        printf("parse_float: Failed to parse float at '%s'\n", *str); // Debug output
        return false; // No valid conversion
    }
    // Check if the parsed value is reasonable (avoid overflow/underflow)
    if (*result > 1e6 || *result < -1e6) {
        printf("parse_float: Value out of range: %f\n", *result); // Debugging
        return false;
    }
    *str = endptr;
    // Skip whitespace and commas
    while (**str == ' ' || **str == ',') (*str)++;
    return true;
}

// Custom parser for PID parameters in format ({kp,ki,kd},{kp,ki,kd}) or ({kp,ki,kd})
static bool parse_pid_params(const char *params, float *values, int expected_count) {
    if (!params || *params != '(') {
        printf("parse_pid_params: Missing opening parenthesis\n"); // Debug output
        return false;
    }
    params++; // Skip '('

    for (int i = 0; i < expected_count; i++) {
        // Check for nested '{' for DualPID
        if (expected_count == 6 && (i == 0 || i == 3)) {
            if (*params != '{') {
                printf("parse_pid_params: Missing opening brace at position %d\n", i); // Debug output
                return false;
            }
            params++; // Skip '{'
        }
        if (!parse_float(&params, &values[i])) { // Corrected: Pass &params and &values[i]
            printf("parse_pid_params: Failed to parse float %d\n", i + 1); // Debug output
            return false;
        }
        // Check for closing '}' for DualPID
        if (expected_count == 6 && (i == 2 || i == 5)) {
            if (*params != '}') {
                printf("parse_pid_params: Missing closing brace at position %d\n", i); // Debug output
                return false;
            }
            params++; // Skip '}'
            if (i == 2 && *params != ',') {
                printf("parse_pid_params: Missing comma after first PID set\n"); // Debug output
                return false;
            }
            if (i == 2) params++; // Skip ','
        }
    }
    // Check for closing ')'
    if (*params != ')') {
        printf("parse_pid_params: Missing closing parenthesis\n"); // Debug output
        return false;
    }
    return true;
}

uint8_t is_cmd_mode(void) {
    return cmd_mode;
}

void strip_brackets(char *str) {
    if (str[0] == '[') {
        memmove(str, str + 1, strlen(str));
    }
    char *end = strchr(str, ']');
    if (end) *end = '\0';
}

void process_command(char *cmd) {
    strip_brackets(cmd);

    if (strcmp(cmd, "CMD") == 0) {
        cmd_mode = 1;
        printf("\r\n\r\nEntered command mode\r\n\r\n");
        main_led(0, 255, 0, 255, 1);
        return;
    }

    if (!cmd_mode) {
        printf("Not in command mode\n");
        return;
    }

    if (strcmp(cmd, "exit") == 0) {
        cmd_mode = 0;
        printf("\r\nExiting command mode, rebooting...\r\n\r\n");
        // Wait for UART transmission to complete
        while (!LL_USART_IsActiveFlag_TC(USART6)); // Ensure transmission complete
        // Busy-wait instead of HAL_Delay
        for (volatile uint32_t i = 0; i < 1000000; i++); // ~200ms at 480MHz
        // Check for pending faults
        if (SCB->HFSR) {
            printf("Hard Fault pending: HFSR=0x%08lX\r\n", SCB->HFSR);
        }
        if (SCB->CFSR) {
            printf("Configurable Fault pending: CFSR=0x%08lX\r\n", SCB->CFSR);
        }
        printf("Disabling interrupts\r\n"); // Debug
        __disable_irq(); // Disable all interrupts
        printf("Triggering NVIC_SystemReset\r\n"); // Debug
        NVIC_SystemReset(); // Primary reset
        printf("NVIC_SystemReset failed, trying SCB reset\r\n"); // Debug (should not reach)
        SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
        printf("SCB reset triggered\r\n"); // Debug (should not reach)
        while (true); // Trap if reset fails
    }
    else if (strcmp(cmd, "cal_accel_gyro") == 0) {
    	  ICM42688P_Calibrate();
    }
    else if (strcmp(cmd, "cal_mag") == 0) {
//        start_mag_calibration(&filter_state);		// Mag-Calibration
    }
    else if (strcmp(cmd, "cal_all") == 0) {
//        start_full_calibration(&filter_state);	// Full-Calibration
    }
    else if (strcmp(cmd, "cancel_cal") == 0) {
//        cancel_calibration();
    }
    else if (strcmp(cmd, "status") == 0) {
        printf("Status OK\n");
    }
    else if (strcmp(cmd, "main_led_ON") == 0) {
        printf("CMD: Executing main_led_ON command\n");
        main_led(0, 255, 0, 255, 1.0);
        main_led_update();
        printf("Main LED ON\n");
    }
    else if (strcmp(cmd, "main_led_OFF") == 0) {
        printf("CMD: Executing main_led_OFF command\n");
        main_led(0, 0, 0, 0, 0);
        main_led_update();
        printf("Main LED OFF\n");
    }
    else if (strncmp(cmd, "set_main_led", 12) == 0) {
        int index, r, g, b;
        float brightness;
        char *params_start = strchr(cmd, '(');
        if (params_start) {
            if (sscanf(params_start, "(%d,%d,%d,%d,%f)", &index, &r, &g, &b, &brightness) == 5) {
                main_led(index, r, g, b, brightness);
                printf("Main LED Set\n");
            } else {
                printf("Invalid Parameters\n");
            }
        }
    }
    else if (strncmp(cmd, "SetRollPID", 10) == 0) {
        float pid_values[6]; // outer_kp, outer_ki, outer_kd, inner_kp, inner_ki, inner_kd
        char *params_start = strchr(cmd, '(');
        if (params_start) {
            if (parse_pid_params(params_start, pid_values, 6)) {
                DualPID_t roll_pid = {
                    .out = {pid_values[0], pid_values[1], pid_values[2]},
                    .in = {pid_values[3], pid_values[4], pid_values[5]}
                };
                if (EEPROM_SetRollPID(&roll_pid) == W25Qxx_OK) {
                    printf("Roll PID Set: Out Kp=%.8f, Ki=%.8f, Kd=%.8f; In Kp=%.8f, Ki=%.8f, Kd=%.8f\n",
                           pid_values[0], pid_values[1], pid_values[2],
                           pid_values[3], pid_values[4], pid_values[5]);
                } else {
                    printf("Failed to set Roll PID\n");
                }
            } else {
                printf("Invalid Roll PID Parameters\n");
            }
        } else {
            printf("Invalid Roll PID Command Format\n");
        }
    }
    else if (strncmp(cmd, "SetPitchPID", 11) == 0) {
        float pid_values[6]; // outer_kp, outer_ki, outer_kd, inner_kp, inner_ki, inner_kd
        char *params_start = strchr(cmd, '(');
        if (params_start) {
            if (parse_pid_params(params_start, pid_values, 6)) {
                DualPID_t pitch_pid = {
                    .out = {pid_values[0], pid_values[1], pid_values[2]},
                    .in = {pid_values[3], pid_values[4], pid_values[5]}
                };
                if (EEPROM_SetPitchPID(&pitch_pid) == W25Qxx_OK) {
                    printf("Pitch PID Set: Out Kp=%.8f, Ki=%.8f, Kd=%.8f; In Kp=%.8f, Ki=%.8f, Kd=%.8f\n",
                           pid_values[0], pid_values[1], pid_values[2],
                           pid_values[3], pid_values[4], pid_values[5]);
                } else {
                    printf("Failed to set Pitch PID\n");
                }
            } else {
                printf("Invalid Pitch PID Parameters\n");
            }
        } else {
            printf("Invalid Pitch PID Command Format\n");
        }
    }
    else if (strncmp(cmd, "SetYawRatePID", 13) == 0) {
        float pid_values[3]; // kp, ki, kd
        char *params_start = strchr(cmd, '(');
        if (params_start) {
            if (parse_pid_params(params_start, pid_values, 3)) {
                PID_t yaw_rate_pid = {pid_values[0], pid_values[1], pid_values[2]};
                if (EEPROM_SetYawRatePID(&yaw_rate_pid) == W25Qxx_OK) {
                    printf("Yaw Rate PID Set: Kp=%.8f, Ki=%.8f, Kd=%.8\n",
                           pid_values[0], pid_values[1], pid_values[2]);
                } else {
                    printf("Failed to set Yaw Rate PID\n");
                }
            } else {
                printf("Invalid Yaw Rate PID Parameters\n");
            }
        } else {
            printf("Invalid Yaw Rate PID Command Format\n");
        }
    }
    else if (strcmp(cmd, "get_roll_pid") == 0) {
        DualPID_t roll_pid;
        if (EEPROM_GetRollPID(&roll_pid) == W25Qxx_OK) {
            printf("Roll PID: Out Kp=%.8f, Ki=%.8f, Kd=%.8f; In Kp=%.8f, Ki=%.8f, Kd=%.8f\n",
                   roll_pid.out.kp, roll_pid.out.ki, roll_pid.out.kd,
                   roll_pid.in.kp, roll_pid.in.ki, roll_pid.in.kd);
        } else {
            printf("Failed to get Roll PID\n");
        }
    }
    else if (strcmp(cmd, "get_pitch_pid") == 0) {
        DualPID_t pitch_pid;
        if (EEPROM_GetPitchPID(&pitch_pid) == W25Qxx_OK) {
            printf("Pitch PID: Out Kp=%.8f, Ki=%.8f, Kd=%.8f; In Kp=%.8f, Ki=%.8f, Kd=%.8f\n",
                   pitch_pid.out.kp, pitch_pid.out.ki, pitch_pid.out.kd,
                   pitch_pid.in.kp, pitch_pid.in.ki, pitch_pid.in.kd);
        } else {
            printf("Failed to get Pitch PID\n");
        }
    }
    else if (strcmp(cmd, "get_yaw_rate_pid") == 0) {
        PID_t yaw_rate_pid;
        if (EEPROM_GetYawRatePID(&yaw_rate_pid) == W25Qxx_OK) {
            printf("Yaw Rate PID: Kp=%.8f, Ki=%.8f, Kd=%.8f\n",
                   yaw_rate_pid.kp, yaw_rate_pid.ki, yaw_rate_pid.kd);
        } else {
            printf("Failed to get Yaw Rate PID\n");
        }
    }
    else {
        printf("Unknown Command\n");
    }
}
