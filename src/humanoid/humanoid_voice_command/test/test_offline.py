"""离线单元测试（不需要 ROS，也不需要 whisper 权重）。

运行::

    pytest test/test_offline.py -v
"""

import os
import sys

import pytest

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, PKG_ROOT)

from humanoid_voice_command.intent import (      # noqa: E402
    DEFAULT_LEXICON,
    INTENTS,
    IntentClassifier,
    hashed_features,
    intent_to_command,
    lcs_ratio,
    load_lexicon,
    normalize_text,
    rule_match,
)

WEIGHTS = os.path.join(PKG_ROOT, 'weights', 'intent_mlp.pt')
SPEED = {'v_forward': 0.30, 'v_backward': 0.20, 'w_turn': 0.60}


def test_normalize_strips_punctuation():
    assert normalize_text('前进。') == '前进'
    assert normalize_text(' 向左转，') == '向左转'
    assert normalize_text('Move forward.') == 'Moveforward'


def test_normalize_folds_repeats():
    # Whisper 短音频的重复幻觉：整段重复应折叠成一份
    assert normalize_text('停止停止停止') == '停止'
    assert normalize_text('左转左转') == '左转'


def test_lcs_ratio_bounds():
    assert lcs_ratio('前进', '前进') == pytest.approx(1.0)
    assert lcs_ratio('前进', '后退') == pytest.approx(0.0)
    assert 0.4 < lcs_ratio('向左转', '左转') < 1.0


@pytest.mark.parametrize('text,intent', [
    ('前进', 'forward'),
    ('往前走一点', 'forward'),
    ('请向前走', 'forward'),
    ('后退', 'backward'),
    ('往后退', 'backward'),
    ('左转', 'turn_left'),
    ('向左转一点', 'turn_left'),
    ('右转', 'turn_right'),
    ('停止', 'stop'),
    ('停下别动', 'stop'),
    ('挥挥手', 'wave'),
    ('快一点', 'faster'),
    ('慢一点', 'slower'),
    ('move forward', 'forward'),
    ('stop', 'stop'),
])
def test_rule_match_keywords(text, intent):
    got, score = rule_match(text, DEFAULT_LEXICON)
    assert got == intent, f'{text!r} -> {got} (score {score:.2f})'
    assert score >= 0.6


def test_rule_match_rejects_unrelated():
    got, score = rule_match('今天天气不错', DEFAULT_LEXICON)
    assert got is None
    assert score < 0.6


@pytest.mark.parametrize('text', ['右', '前', '停', '快', '慢', '左'])
def test_rule_match_rejects_single_char(text):
    """单字歧义太大：ASR 只吐出一个字时不下发任何动作。"""
    got, score = rule_match(text, DEFAULT_LEXICON)
    assert got is None, f'单字 {text!r} 不应命中 {got}'
    assert score == 0.0


@pytest.mark.parametrize('text', ['不要前进', '不要右转', '不用停止', '别往前走', '请不要后退'])
def test_rule_match_rejects_negation(text):
    """否定句不能走子串路径直接下发反方向指令。"""
    got, score = rule_match(text, DEFAULT_LEXICON)
    assert got is None, f'否定句 {text!r} 不应命中 {got}'


def test_rule_match_keeps_negation_words_inside_keywords():
    """「别动」本身就是停止口令，不能当成否定句挡掉。"""
    assert rule_match('别动', DEFAULT_LEXICON)[0] == 'stop'
    assert rule_match('停下别动', DEFAULT_LEXICON)[0] == 'stop'


def test_hashed_features_deterministic():
    a = hashed_features('前进')
    b = hashed_features('前进')
    assert a.shape == (512,)
    assert float((a - b).abs().max()) == 0.0
    assert float(a.norm()) == pytest.approx(1.0, abs=1e-5)


@pytest.mark.skipif(not os.path.exists(WEIGHTS), reason='意图网络权重未生成')
def test_classifier_recovers_asr_error():
    """规则匹配不到、但网络能判对：tiny 把「挥手」识别成「轰手」的真实案例。"""
    clf = IntentClassifier(weights_path=WEIGHTS, lexicon=DEFAULT_LEXICON)
    intent, conf, source = clf.classify('轰手')
    assert intent == 'wave'
    assert source == 'net'
    assert conf > 0.4


@pytest.mark.skipif(not os.path.exists(WEIGHTS), reason='意图网络权重未生成')
def test_classifier_prefers_rule():
    clf = IntentClassifier(weights_path=WEIGHTS, lexicon=DEFAULT_LEXICON)
    intent, conf, source = clf.classify('向前走')
    assert (intent, source) == ('forward', 'rule')
    assert conf == pytest.approx(1.0)


@pytest.mark.skipif(not os.path.exists(WEIGHTS), reason='意图网络权重未生成')
def test_classifier_unknown_for_noise():
    clf = IntentClassifier(weights_path=WEIGHTS, lexicon=DEFAULT_LEXICON)
    intent, _, _ = clf.classify('请把空调调到二十六度')
    assert intent == 'unknown'


@pytest.mark.skipif(not os.path.exists(WEIGHTS), reason='意图网络权重未生成')
@pytest.mark.parametrize('text', ['右', '停', '快'])
def test_classifier_unknown_for_single_char(text):
    """评审指出的漏洞：单字不能绕过拒识闸门直接触发运动。"""
    clf = IntentClassifier(weights_path=WEIGHTS, lexicon=DEFAULT_LEXICON)
    intent, _, source = clf.classify(text)
    assert intent == 'unknown'
    assert source == 'unknown'


@pytest.mark.skipif(not os.path.exists(WEIGHTS), reason='意图网络权重未生成')
@pytest.mark.parametrize('text', ['不要前进', '不要右转', '不用停止'])
def test_classifier_negation_goes_unknown(text):
    """否定句判 unknown，且不走网络（否则网络会硬塞成某个动作）。"""
    clf = IntentClassifier(weights_path=WEIGHTS, lexicon=DEFAULT_LEXICON)
    intent, _, source = clf.classify(text)
    assert intent == 'unknown'
    assert source == 'negated'


@pytest.mark.skipif(not os.path.exists(WEIGHTS), reason='意图网络权重未生成')
def test_classifier_overrides_net_without_char_overlap():
    """「缩转」是「左转」的同音误识：网络判 stop，与输入零重合，用 LCS 候选覆盖。"""
    clf = IntentClassifier(weights_path=WEIGHTS, lexicon=DEFAULT_LEXICON)
    intent, conf, source = clf.classify('缩转')
    assert (intent, source) == ('turn_left', 'lcs')
    assert conf >= 0.45


def test_intent_to_command_mapping():
    assert intent_to_command('forward', 1.0, SPEED)['vx'] == pytest.approx(0.30)
    assert intent_to_command('backward', 1.0, SPEED)['vx'] == pytest.approx(-0.20)
    assert intent_to_command('turn_left', 1.0, SPEED)['wz'] == pytest.approx(0.60)
    assert intent_to_command('turn_right', 1.0, SPEED)['wz'] == pytest.approx(-0.60)
    assert intent_to_command('stop', 1.0, SPEED) == {'vx': 0.0, 'wz': 0.0}
    # 档位倍率线性缩放
    assert intent_to_command('forward', 1.5, SPEED)['vx'] == pytest.approx(0.45)


def test_load_lexicon_covers_all_intents():
    cfg = os.path.join(PKG_ROOT, 'config', 'intents.yaml')
    lexicon = load_lexicon(cfg)
    for intent in INTENTS:
        assert lexicon.get(intent), f'{intent} 缺少关键词'


def test_dedup_repeats_and_cache_dir():
    from humanoid_voice_command.asr import default_cache_dir, dedup_repeats

    assert dedup_repeats('停止, 停止, 停止') == '停止'
    assert dedup_repeats('左转左转') == '左转'
    assert dedup_repeats('前进') == '前进'
    # 仓库不携带 Whisper 权重，缓存目录就是 whisper 的默认位置
    assert default_cache_dir().endswith('whisper')


def test_audio_assets_present():
    audio_dir = os.path.join(PKG_ROOT, 'audio')
    if not os.path.isdir(audio_dir):
        pytest.skip('未放置口令音频')
    wavs = [f for f in os.listdir(audio_dir) if f.endswith('.wav')]
    assert len(wavs) >= 8, f'口令音频只有 {len(wavs)} 条'
