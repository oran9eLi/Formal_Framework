# Latest Firmware

- Build date: `2026-06-11`
- Project: `formal_framework.uvprojx`
- Target: `Target 1`
- Compiler: `ARMCC 5.06 update 7 build 960`
- Result: `0 Error / 0 Warning`
- Code: `34420 bytes`
- RO data: `776 bytes`
- RW data: `236 bytes`
- ZI data: `47332 bytes`

Included artifacts:

- `formal_framework.hex`
- `formal_framework.axf`
- `formal_framework.map`
- `build.log`

This release includes the six-task framework, business EventBus and enabled
Display placeholder service. Business source is grouped under `Business/Inc`
and `Business/Src`. It also includes the first-stage data/status contract and
the application read-only snapshot API. Self-written source functions and
public APIs now have separate purpose comments; vendor and third-party source
remains unchanged. This build also includes the first module registry,
lifecycle, work-item scheduler, and optional task stack/heap serial monitor.
The target is STM32F407ZGTx, matching STM32F407ZGT6. The GNSS legacy generic
driver table, channel samples, status model, and compatibility headers have
been removed. GNSS now exposes only the typed `Init/Service` snapshot API,
which is converted to Framework measurement format by the platform adapter.
The measured Business System stack remains adjusted from 256 to 320 words.
Debug monitoring now uses independent Stack, GNSS, BSP, business, and reserved
module switches. Periodic reports run in a dedicated low-priority DebugTask;
HealthTask no longer formats or transmits diagnostic output. The released
configuration enables boot logging, all registered task Stack/Heap monitoring,
and GNSS periodic monitoring.

Framework no longer includes or initializes Business. `main.c` is the
composition root, and Business initializes its own EventBus and tasks through
`Business_AppInit()`. Business source identifiers now use a typed enum.
Disabled Alarm and Command/ACK queues are no longer allocated from the
FreeRTOS heap.

Task priorities are now configuration macros. SensorTask runs at idle+4,
Estimator at idle+3, Health and Business Acquisition at idle+2. The recovery
monitor call is permanently wired into HealthRun, while GNSS automatic
recovery remains disabled until hardware fault-injection testing is complete.
The Log Pool contract is guarded by `PX4LITE_ENABLE_LOG_POOL=0` and allocates
no runtime resources.

SHA-256:

```text
formal_framework.hex 11574F5001DE3CE726CA7E3BC172933B6DC5162A6C123652CFF888B66543DD4E
formal_framework.axf E6641E76DCC7677480C62710AFF0FEEDB418D603230D9BE7A69DC94A36FD7E82
formal_framework.map 1218AED917424DB8FF066CFD421813F5ECE6F4F962D2F4A983648C21AE17D6AE
build.log 03136FAEC507E407E8EECDFD0AB71655A79D1222FEC998C3C5598E4B58CC61E1
```
