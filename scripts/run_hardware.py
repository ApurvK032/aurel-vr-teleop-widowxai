from widowxai_quest_teleop.hardware import DisabledHardwareBackend, HardwareUnavailableError


def main() -> None:
    try:
        DisabledHardwareBackend().connect()
    except HardwareUnavailableError as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()

