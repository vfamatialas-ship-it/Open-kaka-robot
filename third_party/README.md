# Third-party SDKs

This folder is intentionally lightweight. Large third-party SDKs are not
committed to this repository.

## Damiao motor Python SDK

The current Damiao adapter expects `DM_CAN.py` to be available here:

```text
third_party/damiao/DM_Control_Python/DM_CAN.py
```

Install it with:

```powershell
git clone https://github.com/cmjang/DM_Control_Python.git third_party\damiao\DM_Control_Python
```

If `git` is not available, download the repository as a zip file from GitHub and
extract it to:

```text
third_party/damiao/DM_Control_Python/
```

## Feetech

The current Feetech STS3215 read-only implementation uses the serial protocol
directly and does not require a vendored SDK.
