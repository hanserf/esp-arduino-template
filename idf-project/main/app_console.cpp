/*Console dependencies*/
#include "esp_console.h"
#include "argtable3/argtable3.h"

#include "tcp_console_driver.h"



typedef struct {
    struct arg_int *framesize;
    struct arg_int *quality;
    struct arg_int *brightness;
    struct arg_int *contrast;
    struct arg_int *saturation;
    //struct arg_int *sharpness;
    struct arg_int *special_effect;
    struct arg_int *wb_mode;
    struct arg_int *awb;
    struct arg_int *awb_gain;
    struct arg_int *aec;
    struct arg_int *aec2;
    struct arg_int *ae_level;
    struct arg_int *aec_value;
    struct arg_int *agc;
    struct arg_int *agc_gain;
    struct arg_int *gainceiling;
    struct arg_int *bpc;
    struct arg_int *wpc;
    struct arg_int *raw_gma;
    struct arg_int *lenc;
    struct arg_int *vflip;
    struct arg_int *hmirror;
    struct arg_int *dcw;
    struct arg_int *colorbar;
    struct arg_end *end;
} cam_setting_args_t;
typedef struct {
    struct arg_lit *framesize;
    struct arg_lit *quality;
    struct arg_lit *brightness;
    struct arg_lit *contrast;
    struct arg_lit *saturation;
    struct arg_lit *special_effect;
    struct arg_lit *wb_mode;
    struct arg_lit *awb;
    struct arg_lit *awb_gain;
    struct arg_lit *aec;
    struct arg_lit *aec2;
    struct arg_lit *ae_level;
    struct arg_lit *aec_value;
    struct arg_lit *agc;
    struct arg_lit *agc_gain;
    struct arg_lit *gainceiling;
    struct arg_lit *bpc;
    struct arg_lit *wpc;
    struct arg_lit *raw_gma;
    struct arg_lit *lenc;
    struct arg_lit *vflip;
    struct arg_lit *hmirror;
    struct arg_lit *dcw;
    struct arg_lit *colorbar;
    struct arg_end *end;
} cam_getting_args_t;

static cam_setting_args_t cam_set_args;
static cam_getting_args_t cam_get_args;


static int cam_cmd_allsettings(int argc, char** argv);
static void register_get_all_settings();
static int cam_cmd_camsetting(int argc, char** argv);
static int cam_cmd_camgetting(int argc, char** argv);
static int colsole_exit_cmd(int argc, char** argv);
static void register_exit_command();


static int cam_cmd_allsettings(int argc, char** argv){
    sensor_t * s = esp_camera_sensor_get();
    printf("framesize:%u\n", s->status.framesize);
    printf("quality:%u\n", s->status.quality);
    printf("brightness:%d\n", s->status.brightness);
    printf("contrast:%d\n", s->status.contrast);
    printf("saturation:%d\n", s->status.saturation);
    printf("sharpness:%d\n", s->status.sharpness);
    printf("special_effect:%u\n", s->status.special_effect);
    printf("wb_mode:%u\n", s->status.wb_mode);
    printf("awb:%u\n", s->status.awb);
    printf("awb_gain:%u\n", s->status.awb_gain);
    printf("aec:%u\n", s->status.aec);
    printf("aec2:%u\n", s->status.aec2);
    printf("ae_level:%d\n", s->status.ae_level);
    printf("aec_value:%u\n", s->status.aec_value);
    printf("agc:%u\n", s->status.agc);
    printf("agc_gain:%u\n", s->status.agc_gain);
    printf("gainceiling:%u\n", s->status.gainceiling);
    printf("bpc:%u\n", s->status.bpc);
    printf("wpc:%u\n", s->status.wpc);
    printf("raw_gma:%u\n", s->status.raw_gma);
    printf("lenc:%u\n", s->status.lenc);
    printf("vflip:%u\n", s->status.vflip);
    printf("hmirror:%u\n", s->status.hmirror);
    printf("dcw:%u\n", s->status.dcw);
    printf("colorbar:%u\n", s->status.colorbar);
    return 0;    
}

static void register_get_all_settings()
{
    const esp_console_cmd_t cmd = {
        .command = "get_all_camsettings",
        .help = "Get the current size of free heap memory",
        .hint = NULL,
        .func = &cam_cmd_allsettings,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


void register_app_console(){
    cam_set_args.framesize = arg_int0(NULL, "framesize", "<0-13>", "Imgage Resolution in pixels");
    cam_set_args.quality = arg_int0(NULL, "quality", "<0-63>", "Image quality/compression rate. 0 is highest quality");
    cam_set_args.brightness = arg_int0(NULL, "brightness", "<-2-2>", "Brightness compensation");
    cam_set_args.contrast = arg_int0(NULL, "contrast", "<-2-2>", "Contrast compensation");
    cam_set_args.saturation = arg_int0(NULL, "saturation", "<-2-2>", "Saturation correction");
    cam_set_args.special_effect = arg_int0(NULL, "effects", "<0-6>", "Add special effect filter to image");
    cam_set_args.wb_mode = arg_int0(NULL, "wb_mode", "<0-4>", "White Balance mode");
    cam_set_args.awb = arg_int0(NULL, "awb", "<0|1>", "Enable Auto whitebalance in dsp");
    cam_set_args.awb_gain = arg_int0(NULL, "awb_gain", "<0|1>", "Enable Auto whitebalance gain in dsp");
    cam_set_args.aec = arg_int0(NULL, "aec", "<0|1>", "Enable AutoExposure Control reg");
    cam_set_args.aec2 = arg_int0(NULL, "aec2", "<0|1>", "Select AutoExposure Control reg");
    cam_set_args.ae_level = arg_int0(NULL, "ae_level", "<-2-2>", "Auto Exposure level");
    cam_set_args.aec_value = arg_int0(NULL, "aec_value", "<0-1200>", "Auto Exposure value");
    cam_set_args.agc = arg_int0(NULL, "agc", "<0|1>", "Enable Automatic Gain Control reg");
    cam_set_args.agc_gain = arg_int0(NULL, "agc_gain", "<0-30>", "Set AGC level");
    cam_set_args.gainceiling = arg_int0(NULL, "gainceiling", "<0-30>", "Select AGC Upper Gain Ceiling");
    cam_set_args.bpc = arg_int0(NULL, "bpc", "<0|1>", "Enable Black Pixel correction in dsp");
    cam_set_args.wpc = arg_int0(NULL, "wpc", "<0|1>", "Enable White Pixel correction in dsp");
    cam_set_args.raw_gma = arg_int0(NULL, "raw_gma", "<0|1>", "Enable Gamma curve compensation in dsp");
    cam_set_args.lenc = arg_int0(NULL, "lenc", "<0|1>", "Enable Lense Correction in dsp");
    cam_set_args.vflip = arg_int0(NULL, "vflip", "<0|1>", "Enable vertical image flip");
    cam_set_args.hmirror = arg_int0(NULL, "hmirror", "<0|1>", "Enable Horizontal image mirroring");
    cam_set_args.dcw = arg_int0(NULL, "dcw", "<0|1>", "Enable ?Digital Clock Weighting?, not sure. Relates to pixel clock");
    cam_set_args.colorbar = arg_int0(NULL, "colorbar_test", "<0|1>", "Enable color bar for test pattern");
    cam_set_args.end = arg_end(1);

    cam_get_args.framesize = arg_lit0(NULL, "framesize", "Imgage Resolution in pixels");
    cam_get_args.quality = arg_lit0(NULL, "quality",  "Image quality/compression rate. 0 is highest quality");
    cam_get_args.brightness = arg_lit0(NULL, "brightness",  "Brightness compensation");
    cam_get_args.contrast = arg_lit0(NULL, "contrast",  "Contrast compensation");
    cam_get_args.saturation = arg_lit0(NULL, "saturation", "Saturation correction");
    cam_get_args.special_effect = arg_lit0(NULL, "effects",  "Add special effect filter to image");
    cam_get_args.wb_mode = arg_lit0(NULL, "wb_mode", "White Balance mode");
    cam_get_args.awb = arg_lit0(NULL, "awb",  "Enable Auto whitebalance in dsp");
    cam_get_args.awb_gain = arg_lit0(NULL, "awb_gain", "Enable Auto whitebalance gain in dsp");
    cam_get_args.aec = arg_lit0(NULL, "aec", "Enable AutoExposure Control reg");
    cam_get_args.aec2 = arg_lit0(NULL, "aec2",  "Select AutoExposure Control reg");
    cam_get_args.ae_level = arg_lit0(NULL, "ae_level",  "Auto Exposure level");
    cam_get_args.aec_value = arg_lit0(NULL, "aec_value", "Auto Exposure value");
    cam_get_args.agc = arg_lit0(NULL, "agc", "Enable Automatic Gain Control reg");
    cam_get_args.agc_gain = arg_lit0(NULL, "agc_gain", "Set AGC level");
    cam_get_args.gainceiling = arg_lit0(NULL, "gainceiling", "Select AGC Upper Gain Ceiling");
    cam_get_args.bpc = arg_lit0(NULL, "bpc",  "Enable Black Pixel correction in dsp");
    cam_get_args.wpc = arg_lit0(NULL, "wpc",  "Enable White Pixel correction in dsp");
    cam_get_args.raw_gma = arg_lit0(NULL, "raw_gma",  "Enable Gamma curve compensation in dsp");
    cam_get_args.lenc = arg_lit0(NULL, "lenc",  "Enable Lense Correction in dsp");
    cam_get_args.vflip = arg_lit0(NULL, "vflip",  "Enable vertical image flip");
    cam_get_args.hmirror = arg_lit0(NULL, "hmirror", "Enable Horizontal image mirroring");
    cam_get_args.dcw = arg_lit0(NULL, "dcw",  "Enable ?Digital Clock Weighting?, not sure. Relates to pixel clock");
    cam_get_args.colorbar = arg_lit0(NULL, "colorbar_test", "Enable color bar for test pattern");
    cam_get_args.end = arg_end(1);




    const esp_console_cmd_t cam_set_cmd = {
        .command = "camset",
        .help = "Change camera setting",
        .hint = NULL,
        .func = &cam_cmd_camsetting,
        .argtable = &cam_set_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cam_set_cmd));
    const esp_console_cmd_t cam_get_cmd = {
        .command = "camget",
        .help = "Readout camera setting",
        .hint = NULL,
        .func = &cam_cmd_camgetting,
        .argtable = &cam_get_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cam_get_cmd));

    register_get_all_settings();
    register_exit_command();
}

static int cam_cmd_camsetting(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &cam_set_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, cam_set_args.end, argv[0]);
        return 0;
    }
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;

    
    if(cam_set_args.framesize->count)
    {
        if(s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)cam_set_args.framesize->ival[0]);
    }
    if(cam_set_args.quality->count) res = s->set_quality(s, cam_set_args.quality->ival[0]);
    if(cam_set_args.contrast->count) res = s->set_contrast(s, cam_set_args.contrast->ival[0]);
    if(cam_set_args.brightness->count) res = s->set_brightness(s, cam_set_args.brightness->ival[0]);
    if(cam_set_args.saturation->count) res = s->set_saturation(s, cam_set_args.saturation->ival[0]);
    if(cam_set_args.gainceiling->count) res = s->set_gainceiling(s, (gainceiling_t)cam_set_args.gainceiling->ival[0]);
    if(cam_set_args.colorbar->count) res = s->set_colorbar(s, cam_set_args.colorbar->ival[0]);
    if(cam_set_args.awb->count) res = s->set_whitebal(s, cam_set_args.awb->ival[0]);
    if(cam_set_args.agc->count) res = s->set_gain_ctrl(s, cam_set_args.agc->ival[0]);
    if(cam_set_args.aec->count) res = s->set_exposure_ctrl(s, cam_set_args.aec->ival[0]);
    if(cam_set_args.hmirror->count) res = s->set_hmirror(s, cam_set_args.hmirror->ival[0]);
    if(cam_set_args.vflip->count) res = s->set_vflip(s, cam_set_args.vflip->ival[0]);
    if(cam_set_args.awb_gain->count) res = s->set_awb_gain(s, cam_set_args.awb_gain->ival[0]);
    if(cam_set_args.agc_gain->count) res = s->set_agc_gain(s, cam_set_args.agc_gain->ival[0]);
    if(cam_set_args.aec_value->count) res = s->set_aec_value(s, cam_set_args.aec_value->ival[0]);
    if(cam_set_args.aec2->count) res = s->set_aec2(s, cam_set_args.aec2->ival[0]);
    if(cam_set_args.dcw->count) res = s->set_dcw(s, cam_set_args.dcw->ival[0]);
    if(cam_set_args.bpc->count) res = s->set_bpc(s, cam_set_args.bpc->ival[0]);
    if(cam_set_args.wpc->count) res = s->set_wpc(s, cam_set_args.wpc->ival[0]);
    if(cam_set_args.raw_gma->count) res = s->set_raw_gma(s, cam_set_args.raw_gma->ival[0]);
    if(cam_set_args.lenc->count) res = s->set_lenc(s, cam_set_args.lenc->ival[0]);
    if(cam_set_args.special_effect->count) res = s->set_special_effect(s, cam_set_args.special_effect->ival[0]);
    if(cam_set_args.wb_mode->count) res = s->set_wb_mode(s, cam_set_args.wb_mode->ival[0]);
    if(cam_set_args.ae_level->count) res = s->set_ae_level(s, cam_set_args.ae_level->ival[0]);
    
    return res;
}


static int cam_cmd_camgetting(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &cam_get_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, cam_get_args.end, argv[0]);
        return 0;
    }
    sensor_t * s = esp_camera_sensor_get();
       
    if(cam_get_args.framesize->count) printf("framesize:%u\n", s->status.framesize);
    if(cam_get_args.quality->count) printf("quality:%u\n", s->status.quality);
    if(cam_get_args.contrast->count) printf("contrast:%d\n", s->status.contrast);
    if(cam_get_args.brightness->count) printf("brightness:%d\n", s->status.brightness);
    if(cam_get_args.saturation->count) printf("saturation:%d\n", s->status.saturation);
    if(cam_get_args.gainceiling->count) printf("gainceiling:%u\n", s->status.gainceiling);
    if(cam_get_args.colorbar->count) printf("colorbar:%u\n", s->status.colorbar);
    if(cam_get_args.awb->count) printf("awb:%u\n", s->status.awb);
    if(cam_get_args.agc->count) printf("agc:%u\n", s->status.agc);
    if(cam_get_args.aec->count) printf("aec:%u\n", s->status.aec);
    if(cam_get_args.hmirror->count) printf("hmirror:%u\n", s->status.hmirror);
    if(cam_get_args.vflip->count) printf("vflip:%u\n", s->status.vflip);
    if(cam_get_args.awb_gain->count) printf("awb_gain:%u\n", s->status.awb_gain);
    if(cam_get_args.agc_gain->count) printf("agc_gain:%u\n", s->status.agc_gain);
    if(cam_get_args.aec_value->count) printf("aec_value:%u\n", s->status.aec_value);
    if(cam_get_args.aec2->count) printf("aec2:%u\n", s->status.aec2);
    if(cam_get_args.dcw->count) printf("dcw:%u\n", s->status.dcw);
    if(cam_get_args.bpc->count) printf("bpc:%u\n", s->status.bpc);
    if(cam_get_args.wpc->count) printf("wpc:%u\n", s->status.wpc);
    if(cam_get_args.raw_gma->count) printf("raw_gma:%u\n", s->status.raw_gma);
    if(cam_get_args.lenc->count) printf("lenc:%u\n", s->status.lenc);
    if(cam_get_args.special_effect->count) printf("special_effect:%u\n", s->status.special_effect);
    if(cam_get_args.wb_mode->count) printf("wb_mode:%u\n", s->status.wb_mode);
    if(cam_get_args.ae_level->count) printf("ae_level:%d\n", s->status.ae_level);
    
    return 0;
}

static int colsole_exit_cmd(int argc, char** argv){
    tcp_console_socket_deinit();
    return 0;
}

static void register_exit_command(){
    const esp_console_cmd_t exit_cmd = {
        .command = "exit",
        .help = "Exit Terminal by terminating socket",
        .hint = NULL,
        .func = &colsole_exit_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&exit_cmd));
}