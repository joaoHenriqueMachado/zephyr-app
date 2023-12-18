#include "main.h"

static int generate_fft_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
{
    int fft_signal;

    fft_signal = (int)data;
    
    shell_print(sh, "FFT set to: %s\n" "Value sent to DAC driver: %d\n", argv[0], fft_signal);
    return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_fft, generate_fft_cmd_handler, (sine, 1, "FFT_Sine"), (cosine, 2, "FFT_Cosine"));
SHELL_CMD_REGISTER(fft, &sub_fft, "Set FFT Signal", NULL);