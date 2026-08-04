from __future__ import annotations

from textwrap import dedent


_THEMES = {
    "petmind": {
        "max_width": 780,
        "background_tail": "#fff8f0",
        "accent_rgb": "232,123,53",
        "button_start": "#d35400",
        "button_end": "#e87b35",
    },
    "panda": {
        "max_width": 800,
        "background_tail": "#f1f8e9",
        "accent_rgb": "46,125,50",
        "button_start": "#2e7d32",
        "button_end": "#43a047",
    },
    "livestock": {
        "max_width": 800,
        "background_tail": "#fff8e1",
        "accent_rgb": "141,110,99",
        "button_start": "#6d4c41",
        "button_end": "#8d6e63",
    },
}


def render_feedback_widget_css(theme: str) -> str:
    cfg = _THEMES[theme]
    return dedent(
        f"""
        /* Feedback rating bar */
        .feedback-bar{{max-width:{cfg["max_width"]}px;width:100%;margin:6px auto 0;background:linear-gradient(135deg,var(--accent-light),{cfg["background_tail"]});
                      border:1px solid var(--border);border-radius:var(--radius);padding:12px 16px;
                      display:grid;grid-template-columns:auto auto minmax(180px,1fr) auto;align-items:center;gap:10px;
                      animation:slideUp .35s ease both;box-shadow:var(--shadow-sm)}}
        .feedback-bar .fb-label{{font-size:13px;color:var(--text2);white-space:nowrap}}
        .star-group{{display:grid;grid-template-columns:repeat(5,44px);gap:8px;align-items:center;justify-content:start;position:relative;z-index:1}}
        .star{{display:inline-flex;align-items:center;justify-content:center;width:44px;height:44px;
              cursor:pointer;background:#fff;border:1px solid rgba(251,191,36,.18);font-size:24px;color:#d1d5db;
              transition:color .15s,border-color .15s,box-shadow .15s,background-color .15s;padding:0;line-height:1;
              -webkit-appearance:none;appearance:none;font-family:inherit;border-radius:999px;
              touch-action:manipulation;-webkit-tap-highlight-color:transparent}}
        .star:hover,.star.active,.star.hovered{{color:#fbbf24;background:#fffaf0;border-color:rgba(251,191,36,.45);
              box-shadow:0 0 0 2px rgba(251,191,36,.12)}}
        .fb-comment{{flex:1 1 200px;resize:none;border:1.5px solid var(--border);border-radius:10px;
                    padding:8px 12px;font-size:13px;font-family:inherit;min-height:36px;max-height:60px;
                    min-width:0;width:100%;
                    outline:none;transition:border-color .2s,box-shadow .2s}}
        .fb-comment:focus{{border-color:var(--accent);box-shadow:0 0 0 3px rgba({cfg["accent_rgb"]},.1)}}
        .fb-submit{{padding:8px 20px;background:linear-gradient(135deg,{cfg["button_start"]},{cfg["button_end"]});color:#fff;border:none;
                   border-radius:10px;font-size:13px;cursor:pointer;font-weight:600;white-space:nowrap;
                   transition:all .2s;box-shadow:0 2px 6px rgba({cfg["accent_rgb"]},.15)}}
        .fb-submit:disabled{{opacity:.4;cursor:not-allowed;box-shadow:none}}
        .fb-submit:hover:not(:disabled){{transform:translateY(-1px);box-shadow:0 4px 12px rgba({cfg["accent_rgb"]},.25)}}
        .fb-done{{font-size:13px;color:var(--accent);font-weight:600;animation:fadeSlideIn .3s ease both}}
        @media(max-width:600px){{
        .feedback-bar{{grid-template-columns:1fr;padding:10px 12px;gap:8px}}
        .feedback-bar .fb-label{{white-space:normal}}
        .star-group{{grid-template-columns:repeat(5,minmax(44px,1fr));width:100%;gap:8px}}
        .star{{width:100%;height:44px;font-size:24px}}
        .fb-comment{{min-height:44px;max-height:96px}}
        .fb-submit{{width:100%;min-height:40px}}
        }}
        """
    ).strip()


FEEDBACK_WIDGET_JS = dedent(
    """
    function showFeedback(msgId, reqId) {
      const bar = document.createElement('div');
      bar.className = 'feedback-bar';
      bar.id = msgId + '-fb';
      let selectedRating = 0;

      const starGroupHtml = [1,2,3,4,5].map(n =>
        '<button type="button" class="star" data-n="' + n + '" title="' + n + ' 星">★</button>'
      ).join('');

      bar.innerHTML = '<span class="fb-label">回答质量如何？</span>' +
        '<div class="star-group">' + starGroupHtml + '</div>' +
        '<textarea class="fb-comment" placeholder="补充评论（可选）" rows="1"></textarea>' +
        '<button type="button" class="fb-submit" disabled>提交</button>';

      const stars = bar.querySelectorAll('.star');

      stars.forEach(s => {
        s.addEventListener('mouseenter', () => {
          const n = parseInt(s.dataset.n);
          stars.forEach((st, idx) => st.classList.toggle('hovered', idx < n));
        });
        s.addEventListener('mouseleave', () => {
          stars.forEach(st => st.classList.remove('hovered'));
        });
        s.addEventListener('click', () => {
          selectedRating = parseInt(s.dataset.n);
          stars.forEach((st, idx) => st.classList.toggle('active', idx < selectedRating));
          bar.querySelector('.fb-submit').disabled = false;
        });
      });

      bar.querySelector('.fb-submit').addEventListener('click', async function() {
        if (!selectedRating) return;
        this.disabled = true;
        this.textContent = '提交中...';
        const comment = bar.querySelector('.fb-comment').value.trim();
        try {
          const r = await fetch('/qa/feedback', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ request_id: reqId, rating: selectedRating, comment })
          });
          if (r.ok) {
            bar.innerHTML = '<span class="fb-done">感谢你的反馈！</span>';
          } else {
            const d = await r.json().catch(() => ({}));
            bar.innerHTML = '<span class="fb-done" style="color:var(--text2)">' + (d.detail || '提交失败') + '</span>';
          }
        } catch (e) {
          bar.innerHTML = '<span class="fb-done" style="color:var(--text2)">网络错误</span>';
        }
      });

      const msgEl = document.getElementById(msgId);
      if (msgEl) msgEl.after(bar);
      scroll();
    }
    """
).strip()


__all__ = ["FEEDBACK_WIDGET_JS", "render_feedback_widget_css"]
