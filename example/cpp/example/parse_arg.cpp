#include "test.hpp"

#include <fstream>
#include <iostream>

#define LOCAL static inline

/* #region parse_opt */

LOCAL void help(const char* exec)
{
    std::cout << "Usage: " << exec << " [OPTION]" << std::endl;
    std::cout << "  -h,--help           Display this information" << std::endl;
    std::cout << "  -v,--version        Display the version of the program" << std::endl;
    std::cout << "  -d,--device NUM     Set the device number" << std::endl;
    std::cout << "  --raw/--depth       Display the raw or depth frame" << std::endl;
    std::cout << "  -P,--no-preview     Do not display the preview" << std::endl;
    std::cout << "  -C,--no-confidence  Do not display the confidence" << std::endl;
    std::cout << "  -A,--no-amplitude   Do not display the amplitude" << std::endl;
    std::cout << "  --fps FPS           Set the fps of the camera" << std::endl;
    std::cout << "  --mode MODE         Set the mode of the camera" << std::endl;
    std::cout << "      0               320 * 240 with single frequency" << std::endl;
    std::cout << "      1               320 * 240 with double frequency" << std::endl;
    std::cout << "      2               640 * 480 with single frequency" << std::endl;
    std::cout << "      3(*)            640 * 480 with double frequency (default)" << std::endl;
    std::cout << "  --cfg PATH          The usb camera config file path" << std::endl;
    std::cout << "  -m,--min-range NUM  Set the min range of the camera (mm)" << std::endl;
    std::cout << "  -M,--max-range NUM  Set the max range of the camera (mm)" << std::endl;
}

enum class ArgEnum {
    none,
    help,
    version,
    device,
    raw,
    depth,
    no_preview,
    no_confidence,
    no_amplitude,
    fps,
    mode,
    cfg,
    min_range,
    max_range,
};

template <ArgEnum arg_enum> LOCAL const char* to_str()
{
    switch (arg_enum) {
    case ArgEnum::help:
        return "help";
    case ArgEnum::version:
        return "version";
    case ArgEnum::device:
        return "device";
    case ArgEnum::raw:
        return "raw";
    case ArgEnum::depth:
        return "depth";
    case ArgEnum::no_preview:
        return "no-preview";
    case ArgEnum::no_confidence:
        return "no-confidence";
    case ArgEnum::no_amplitude:
        return "no-amplitude";
    case ArgEnum::fps:
        return "fps";
    case ArgEnum::mode:
        return "mode";
    case ArgEnum::cfg:
        return "cfg";
    case ArgEnum::min_range:
        return "min_range";
    case ArgEnum::max_range:
        return "max_range";
    default:
        return "unknown";
    }
}

LOCAL bool __parse_cfg(opt_data& data, const char* path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    data.cfg = path;
    return true;
}

template <ArgEnum arg_enum> LOCAL int __parse_opt(const char* exec, const char* curr, const char* opt, opt_data& data)
{
    std::cout << "[info] process " << to_str<arg_enum>() << std::endl;
    switch (arg_enum) {
    case ArgEnum::device: {
        if (opt == nullptr) {
            std::cerr << "Invalid device number" << std::endl;
            return 0;
        }
        data.device = atoi(opt);
        return 2;
    } break;
    case ArgEnum::cfg: {
        if (opt == nullptr) {
            std::cerr << "Invalid config file path" << std::endl;
            return 0;
        }
        if (!__parse_cfg(data, opt)) {
            std::cerr << "Invalid config file path" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::raw: {
        data.raw = true;
    } break;
    case ArgEnum::depth: {
        data.raw = false;
    } break;
    case ArgEnum::no_preview: {
        data.no_preview = true;
    } break;
    case ArgEnum::no_confidence: {
        data.no_confidence = true;
    } break;
    case ArgEnum::no_amplitude: {
        data.no_amplitude = true;
    } break;
    case ArgEnum::min_range: {
        if (opt == nullptr) {
            std::cerr << "Invalid min range" << std::endl;
            return 0;
        }
        data.min_range = atoi(opt);
        if (data.min_range < 0) {
            std::cerr << "Invalid min range" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::max_range: {
        if (opt == nullptr) {
            std::cerr << "Invalid max range" << std::endl;
            return 0;
        }
        data.max_range = atoi(opt);
        if (data.max_range < 0) {
            std::cerr << "Invalid max range" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::mode: {
        if (opt == nullptr) {
            std::cerr << "Invalid mode" << std::endl;
            return 0;
        }
        data.mode = atoi(opt);
        if (data.mode < 0 || data.mode > 4) {
            std::cerr << "Invalid mode" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::fps: {
        if (opt == nullptr) {
            std::cerr << "Invalid fps" << std::endl;
            return 0;
        }
        data.fps = atoi(opt);
        if (data.fps < 0) {
            std::cerr << "Invalid fps" << std::endl;
            return 0;
        }
        return 2;
    } break;
    case ArgEnum::help: {
        help(exec);
        exit(0);
    } break;
    case ArgEnum::version: {
        std::cout << "Version: 1.0.0" << std::endl;
        exit(0);
    } break;
    }
    return 1;
}

LOCAL int __parse_opt(int argc, char* argv[], int curr, opt_data& data)
{
    bool is_opt;
    const char* arg = argv[curr];
    const char* next = curr + 1 < argc ? argv[curr + 1] : nullptr;
    // if start with "--"
    if (arg[0] != '-')
        return 0;
    if (arg[1] == '-') {
        is_opt = true;
        arg += 2;
    } else {
        is_opt = false;
        arg += 1;
    }

// parse a short option
#define SHORT(chr, type)                                                                                               \
    case chr:                                                                                                          \
        if (1 != __parse_opt<type>(argv[0], arg, next, data)) {                                                        \
            return 0;                                                                                                  \
        }                                                                                                              \
        break;
// parse a short option with argument
#define SHORT_END(chr, type)                                                                                           \
    case chr:                                                                                                          \
        if (arg[index + 1] != '\0') {                                                                                  \
            return __parse_opt<type>(argv[0], arg, arg + index + 1, data) == 0 ? 0 : 1;                                \
        }                                                                                                              \
        return __parse_opt<type>(argv[0], arg, next, data);                                                            \
        break;
// parse a long option
#define LONG(chr, type)                                                                                                \
    else if (!strcmp(arg, chr))                                                                                        \
    {                                                                                                                  \
        return __parse_opt<type>(argv[0], arg, next, data);                                                            \
    }

    if (!is_opt) {
        for (int index = 0; arg[index] != '\0'; index++) {
            switch (arg[index]) {
                // SHORT('r', ArgEnum::raw)
                SHORT('h', ArgEnum::help)
                SHORT('v', ArgEnum::version)
                SHORT('P', ArgEnum::no_preview)
                SHORT('C', ArgEnum::no_confidence)
                SHORT('A', ArgEnum::no_amplitude)
                SHORT_END('d', ArgEnum::device)
                SHORT_END('m', ArgEnum::min_range)
                SHORT_END('M', ArgEnum::max_range)
            default:
                return 0;
            }
        }
    } else {
        if (!strcmp(arg, "")) {
            return 0;
        }
        LONG("cfg", ArgEnum::cfg)
        LONG("raw", ArgEnum::raw)
        LONG("depth", ArgEnum::depth)
        LONG("no-preview", ArgEnum::no_preview)
        LONG("no-confidence", ArgEnum::no_confidence)
        LONG("no-amplitude", ArgEnum::no_amplitude)
        LONG("min-range", ArgEnum::min_range)
        LONG("max-range", ArgEnum::max_range)
        LONG("mode", ArgEnum::mode)
        LONG("fps", ArgEnum::fps)
        LONG("help", ArgEnum::help)
        LONG("version", ArgEnum::version)
        else
        {
            return 0;
        }
    }

#undef SHORT
#undef SHORT_END
#undef LONG

    return 1;
}

bool parse_opt(int argc, char* argv[], opt_data& opt)
{
    for (int i = 1; i < argc;) {
        int tmp = __parse_opt(argc, argv, i, opt);
        if (tmp == 0) {
            std::cerr << "Invalid option: " << argv[i] << std::endl;
            return false;
        }
        i += tmp;
    }

    // check
    if (opt.no_confidence && opt.raw) {
        std::cerr << "Invalid option: --no-confidence and --raw are exclusive" << std::endl;
        return false;
    }
    if (opt.no_amplitude && opt.raw) {
        std::cerr << "Invalid option: --no-amplitude and --raw are exclusive" << std::endl;
        return false;
    }

    return true;
}

/* #endregion */
