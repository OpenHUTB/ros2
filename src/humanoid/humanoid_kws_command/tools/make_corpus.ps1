# 用 Windows 自带的 System.Speech 合成 KWS 语料（不需要联网）。
# 用法：powershell -ExecutionPolicy Bypass -File tools/make_corpus.ps1 -OutDir corpus_wav
param([string]$OutDir = "corpus_wav")

Add-Type -AssemblyName System.Speech
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$syn = New-Object System.Speech.Synthesis.SpeechSynthesizer
$voice = $syn.GetInstalledVoices() | Where-Object { $_.VoiceInfo.Culture.Name -eq "zh-CN" } | Select-Object -First 1
if ($null -eq $voice) { throw "系统没有中文语音，无法生成语料" }
$syn.SelectVoice($voice.VoiceInfo.Name)
$fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(16000, `
    [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, `
    [System.Speech.AudioFormat.AudioChannel]::Mono)

$words = @(
  @("forward","前进"), @("backward","后退"), @("turn_left","左转"), @("turn_right","右转"),
  @("stop","停止"), @("faster","加速"), @("slower","减速"), @("wave","挥手"),
  @("crouch","蹲下"), @("stand_up","起立"))
$unknown = @(
  @("hello","你好"), @("weather","天气"), @("music","音乐"), @("light_on","开灯"),
  @("thanks","谢谢"), @("open_door","开门"))
$pitch = @("-15%", "+0%", "+15%", "+30%")
$rate = @("0.85", "1.0", "1.2")

$n = 0
foreach ($set in @(@($words, "cmd"), @($unknown, "unk"))) {
  foreach ($w in $set[0]) {
    foreach ($p in $pitch) {
      foreach ($r in $rate) {
        $tag = ($p -replace "[^0-9-]", "") + "_" + $r
        $file = Join-Path $OutDir ("{0}__{1}__{2}.wav" -f $set[1], $w[0], $tag)
        $ssml = "<speak version='1.0' xmlns='http://www.w3.org/2001/10/synthesis' xml:lang='zh-CN'><prosody pitch='$p' rate='$r'>$($w[1])</prosody></speak>"
        $syn.SetOutputToWaveFile($file, $fmt)
        $syn.SpeakSsml($ssml)
        $syn.SetOutputToNull()
        $n++
      }
    }
  }
}
$syn.Dispose()
Write-Host "已生成 $n 个 wav 到 $OutDir（16 kHz / 16-bit / 单声道）"
