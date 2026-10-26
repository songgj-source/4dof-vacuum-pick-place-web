"""Spoken and saved place names may differ only in spacing."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base_drive import BaseError, find_label


@pytest.mark.parametrize('spoken,saved', [
    ('시작 위치', '시작위치'), ('시작위치', '시작 위치'),
    ('  시작   위치  ', '시작위치'), ('시작\t위치', '시작위치'),
])
def test_spacing_variants_resolve_to_the_saved_id(spoken, saved):
    labels = [dict(id='desk', name='책상'), dict(id='start', name=saved)]
    assert find_label(spoken, labels)['id'] == 'start'


def test_original_exact_name_wins_when_both_names_exist():
    labels = [dict(id='spaced', name='시작 위치'), dict(id='compact', name='시작위치')]
    assert find_label('시작 위치', labels)['id'] == 'spaced'
    assert find_label('시작위치', labels)['id'] == 'compact'


def test_ambiguous_spacing_match_never_selects_arbitrarily():
    labels = [dict(id='spaced', name='시작 위치'), dict(id='compact', name='시작위치')]
    with pytest.raises(BaseError, match='여럿'):
        find_label('시 작 위치', labels)


def test_normalized_exact_match_wins_over_partial_match():
    labels = [dict(id='other', name='시작 위치 옆'), dict(id='start', name='시작위치')]
    assert find_label('시작 위치', labels)['id'] == 'start'


@pytest.mark.parametrize('name', ['', '   ', '없는 장소'])
def test_empty_or_unknown_place_still_fails(name):
    with pytest.raises(BaseError, match='지도에 없습니다'):
        find_label(name, [dict(id='desk', name='책상')])
