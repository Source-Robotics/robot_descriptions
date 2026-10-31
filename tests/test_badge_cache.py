import copy

from shapely.geometry import box

import build_side_badge as badge


def test_badge_cache_preserves_pixels_and_invalidates_on_palette_source_or_corruption(tmp_path,monkeypatch):
    monkeypatch.setattr(badge,'CONFIG',tmp_path/'badge.json')
    source=tmp_path/'badge.svg';source.write_text('<svg/>')
    config={'source':'badge.svg','texture_width_px':64,'texture_height_px':16,
            'panel_color':'#CAD1EE','colors':{part:'#464545' for part in badge.PARTS}}
    shapes={part:box(1,1,2,2) for part in badge.PARTS};viewport=[0.,0.,4.,4.]
    original=badge.cairosvg.svg2png;calls=[]
    def render(**kwargs):
        calls.append(1);return original(**kwargs)
    monkeypatch.setattr(badge.cairosvg,'svg2png',render)
    first=badge.render_texture(config,shapes,viewport)
    assert len(calls)==1
    assert badge.render_texture(config,shapes,viewport)==first
    assert len(calls)==1
    changed=copy.deepcopy(config);changed['panel_color']='#FFFFFF'
    assert badge.render_texture(changed,shapes,viewport)!=first
    assert len(calls)==2
    source.write_text('<svg><!-- changed source --></svg>')
    badge.render_texture(changed,shapes,viewport)
    assert len(calls)==3
    (tmp_path/'texture_cache/par6_badge.png').write_bytes(b'corrupt')
    badge.render_texture(changed,shapes,viewport)
    assert len(calls)==4
