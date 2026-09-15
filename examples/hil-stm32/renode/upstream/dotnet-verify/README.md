# dotnet-verify:不建整個 Renode,對 1.16.1 的二進位組件驗證上游修正

Renode 從原始碼建要 .NET SDK + tlib + 約 10 GB、20 分鐘以上。這裡只做兩件事,幾分鐘:

1. `UpstreamCompile.csproj`:把 fork 上的 `STM32_Timer.cs`(類名不變)對 `/opt/renode/bin/Infrastructure.dll` 編譯 → 證明它在 1.16.1 的 API 下能編(0 warning)。
2. `TimerTests.csproj`:`STM32_Timer_Fixed.cs` + `STM32_TimerTests.cs`(類名替換成 `_Fixed`)用 NUnit 跑 → 4/4 綠;同一份測試對原版 `STM32_Timer` → 4/4 紅。

`Machine()` 會經 Mono.Unix 碰檔案系統,測試輸出目錄要帶進 Renode 自帶的組件與 `runtimes/linux-x64/native/*.so`(csproj 裡的 `<None Include>`),否則 `DllNotFoundException: Mono.Unix`。

```bash
# 從映像抽出 /opt/renode/bin 到 ../renode-bin(不進 repo)
cid=$(docker create antmicro/renode:latest); docker cp $cid:/opt/renode/bin/. renode-bin/; docker rm $cid
# restore 要網路(NUnit 套件);之後 --network none
docker run --rm --network bridge -v "$PWD":/w -w /w -e HOME=/tmp -e DOTNET_CLI_HOME=/tmp -e NUGET_PACKAGES=/w/.nuget \
  mcr.microsoft.com/dotnet/sdk:8.0 sh -c 'dotnet build upstream-compile -c Release && dotnet test tests -c Release'
```
