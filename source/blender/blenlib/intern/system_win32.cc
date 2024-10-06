/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include <Windows.h>
#include <commctrl.h>
#include <iomanip>
#include <sstream>

#include "BLI_utildefines.h"

#include "BLI_system.h"

/* -------------------------------------------------------------------- */
/** \name showMessageBox
 * \{ */

static std::string url_encode(const char *str)
{
  std::ostringstream encoded;
  while (char c = *str++) {
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded << c;
    }
    else if (c == ' ') {
      encoded << '+';
    }
    else {
      encoded << '%' << std::setw(2) << std::setfill('0') << std::hex << std::uppercase
              << int((unsigned char)c);
    }
  }
  return encoded.str();
}

static std::string get_os_info()
{
  OSVERSIONINFOEX osvi;
  ZeroMemory(&osvi, sizeof(OSVERSIONINFOEX));
  osvi.dwOSVersionInfoSize = sizeof(OSVERSIONINFOEX);
  if (!GetVersionEx((OSVERSIONINFO *)&osvi)) {
    return "Unknown System";
  }

  std::string version = std::to_string(osvi.dwMajorVersion) + "-" +
                        std::to_string(osvi.dwMajorVersion) + "." +
                        std::to_string(osvi.dwMinorVersion) + "." +
                        std::to_string(osvi.dwBuildNumber) + "-SP" +
                        std::to_string(osvi.wServicePackMajor);

  SYSTEM_INFO si;
  GetSystemInfo(&si);
  std::string architecture;
  switch (si.wProcessorArchitecture) {
    case PROCESSOR_ARCHITECTURE_AMD64:
      architecture = "64 Bits";
      break;
    case PROCESSOR_ARCHITECTURE_INTEL:
      architecture = "32 Bits";
      break;
    case PROCESSOR_ARCHITECTURE_ARM:
      architecture = "ARM Architecture";
      break;
    case PROCESSOR_ARCHITECTURE_ARM64:
      architecture = "ARM64 Architecture";
      break;
    case PROCESSOR_ARCHITECTURE_ARM32_ON_WIN64:
      architecture = "ARM32 on Windows 64-bit";
      break;
    case PROCESSOR_ARCHITECTURE_IA32_ON_ARM64:
      architecture = "IA32 on ARM64";
      break;
    default:
      architecture = "Unknown Architecture";
  }

  return "Windows-" + version + " " + architecture;
}

/**
 * Displays a crash popup with options to open the crash log and report a bug.
 * This is based on the `showMessageBox` function in `GHOST_SystemWin32.cc`.
 */
static void showMessageBox(const char *message,
                           const char *filepath,
                           const char *gpu_name,
                           const char *build_version)
{
  /* InitCommonControls is called during GHOST System initialization, so this is redundant. */
  // InitCommonControls();

  std::wstring full_message_16 =
      L"The application encountered a fatal error and will close.\n\n"
      L"If you know the steps to reproduce this crash, please file a bug report.\n\n"
      L"The crash log can be found at:\n" +
      std::wstring(filepath, filepath + strlen(filepath)) + L"\n\n" +
      std::wstring(message, message + strlen(message));

  TASKDIALOGCONFIG config = {0};
  const TASKDIALOG_BUTTON buttons[] = {
      {IDOK, L"Report a Bug"}, {IDHELP, L"Open Crash Log"}, {IDCLOSE, L"Close"}};

  config.cbSize = sizeof(config);
  config.hInstance = 0;
  config.dwCommonButtons = 0;
  config.pszMainIcon = TD_ERROR_ICON;
  config.pszWindowTitle = L"Blender";
  config.pszMainInstruction = L"Blender has crashed!";
  config.pszContent = full_message_16.c_str();
  config.pButtons = buttons;
  config.cButtons = ARRAY_SIZE(buttons);

  /* Lambda callback for handling button events. */
  const char *data[] = {filepath, gpu_name, build_version};
  config.lpCallbackData = reinterpret_cast<LONG_PTR>(data);
  config.pfCallback = [](HWND /*hwnd*/,
                         UINT uNotification,
                         WPARAM wParam,
                         LPARAM /*lParam*/,
                         LONG_PTR dwRefData) -> HRESULT {
    if (uNotification != TDN_BUTTON_CLICKED) {
      return S_OK;
    }
    char *const *data_ptr = reinterpret_cast<char *const *>(dwRefData);
    const char *data_filepath = data_ptr[0];
    const char *data_gpu_name = data_ptr[1];
    const char *data_build_version = data_ptr[2];

    int pnButton = static_cast<int>(wParam);
    switch (pnButton) {
      case IDCLOSE:
        return S_OK;
      case IDHELP:
        /* Open the crash log. */
        ShellExecute(nullptr, "open", data_filepath, nullptr, nullptr, SW_SHOWNORMAL);
        return S_FALSE;
      case IDOK: {
        /* clang-format off */
        std::string link =
            "https://redirect.blender.org/"
            "?type=bug_report"
            "&project=blender"
            "&os=" + url_encode(get_os_info().c_str()) +
            "&gpu=" + url_encode(data_gpu_name) +
            "&broken_version=" + url_encode(data_build_version);
        /* clang-format on */
        ShellExecute(nullptr, "open", link.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
        return S_FALSE;
      }
      default:
        return S_FALSE;
    }
  };

  TaskDialogIndirect(&config, nullptr, nullptr, nullptr);
}

void BLI_windows_exception_show_dialog(const char *filepath,
                                       const char *gpu_name,
                                       const char *build_version)
{
  char message[512];
  bli_windows_exception_message_get(message);
  showMessageBox(message, filepath, gpu_name, build_version);
  fprintf(stderr, message);
  fflush(stderr);
}

/** \} */
