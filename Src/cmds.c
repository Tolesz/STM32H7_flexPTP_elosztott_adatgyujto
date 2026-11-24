#include "cmds.h"

#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include "cliutils/cli.h"
#include "standard_output/standard_output.h"

#include "flexptp/task_ptp.h"
#include "ptp_tim_sync.h"
#include "capture_handler.h"

// ---------------------------------

CMD_FUNCTION(os_info) {
    MSG("OS: FreeRTOS\nKernel version: %s\n", tskKERNEL_VERSION_NUMBER);

    HeapStats_t stats;
    vPortGetHeapStats(&stats);
    MSG("Free OS memory: %u bytes\n", stats.xAvailableHeapSpaceInBytes);

    return 0;
}

CMD_FUNCTION(start_flexptp) {
    if (!task_ptp_is_operating()) {
        MSG("Starting flexPTP...\n\n");
        reg_task_ptp();
    } else {
        MSG("Nice try, but no. :)\n"
            "flexptp is already up and running!\n");
    }

    return 0;
}

CMD_FUNCTION(meas_gate) {
    if (argc >= 1) {
        Capture_SetGateMs((uint32_t)atoi(ppArgs[0]));
    }
    MSG("Kapuido: %u ms\n", Capture_GetGateMs());
    return 0;
}

CMD_FUNCTION(meas_channel) {
    if (argc >= 2) {
        int ch = atoi(ppArgs[0]);
        if (ch < 1 || ch > CAPTURE_CHANNEL_COUNT) {
            MSG("Ervenytelen csatorna! (1..%u)\n", CAPTURE_CHANNEL_COUNT);
            return -1;
        }
        if (!strcmp(ppArgs[1], "on")) {
            Capture_SetChannelEnabled(ch, true);
        } else if (!strcmp(ppArgs[1], "off")) {
            Capture_SetChannelEnabled(ch, false);
        } else {
            return -1;
        }
    }

    for (uint32_t ch = 1; ch <= CAPTURE_CHANNEL_COUNT; ch++) {
        MSG("CH%u: %s\n", ch, Capture_IsChannelEnabled(ch) ? "be" : "ki");
    }
    return 0;
}

// ---------------------------------

void cmd_init() {
    cli_register_command("osinfo \t\t\tPrint OS-related information", 1, 0, os_info);
    cli_register_command("flexptp \t\t\tStart flexPTP daemon", 1, 0, start_flexptp);
    cli_register_command("gate {ms} \t\t\tSet or query frequency counter gate time [ms]", 1, 0, meas_gate);
    cli_register_command("meas {ch} {on|off} \t\t\tEnable/disable or query measurement channels", 1, 0, meas_channel);
    PtpTimSync_RegisterCli();
}