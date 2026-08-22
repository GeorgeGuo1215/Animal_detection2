"""为三端联调在 petmind MySQL 里插入一只与 PetHealth 宠物 id 对齐的动物 + 生理数据。

PetHealth 的宠物 id 是 cuid（如 cmr1t462i00011sv8bjr1ixws），而 agent 的
sql.search / vitals.summary 读的是自己的 petmind 库，按 animal_id 过滤。这里把该
cuid 作为 animal_id 写入 animals/sensor_events/vitals_samples/temp_samples/daily_reports，
使 MoE 的物种软过滤、体征摘要、日报查询都能命中真实数据。

幂等：可重复运行（唯一键 upsert / INSERT IGNORE）。

用法：
    python scripts/seed_integration_animal.py <animal_id> <species>
    # 默认 species=cat
"""
from __future__ import annotations

import sys

import pymysql

# 与 app/integrations/petmind_mysql/config.py 默认一致（本地 petmind，空密码）
DB = dict(host="127.0.0.1", port=3306, user="root", password="", database="petmind", charset="utf8mb4")

# 猫静息参考：HR 140-180、RR 20-30、体温 38.1-39.2
VITALS = [
    (0, 152, 24),
    (1, 158, 26),
    (2, 149, 22),
    (3, 165, 28),
    (4, 156, 25),
    (5, 161, 27),
    (6, 147, 23),
]
TEMPS = [
    (0, 38.4),
    (1, 38.6),
    (2, 38.5),
    (3, 38.9),
    (4, 38.7),
    (5, 38.8),
    (6, 38.5),
]


def main() -> None:
    """向 petmind 库幂等写入联调用动物画像与体征样本。"""
    animal_id = sys.argv[1] if len(sys.argv) > 1 else "cmr1t462i00011sv8bjr1ixws"
    species = sys.argv[2] if len(sys.argv) > 2 else "cat"
    event_id = f"integ_evt_{animal_id}_1"

    conn = pymysql.connect(**DB, autocommit=True)
    try:
        with conn.cursor() as cur:
            # 1) animals（种子画像，驱动物种软过滤）
            cur.execute(
                """
                INSERT INTO animals (animal_id, species, name, breed, sex, age_months, weight_kg)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                ON DUPLICATE KEY UPDATE species=VALUES(species), name=VALUES(name),
                    breed=VALUES(breed), sex=VALUES(sex), age_months=VALUES(age_months),
                    weight_kg=VALUES(weight_kg)
                """,
                (animal_id, species, "测试猫", "中华田园猫", "unknown", 24, 4.20),
            )

            # 2) sensor_events（一个采集窗口，ts=最近，raw_payload 必填）
            cur.execute(
                """
                INSERT INTO sensor_events (event_id, ts, timezone, animal_id, raw_payload)
                VALUES (%s, NOW(3), %s, %s, %s)
                ON DUPLICATE KEY UPDATE ts=NOW(3)
                """,
                (event_id, "+08:00", animal_id, "{}"),
            )
            cur.execute("SELECT id FROM sensor_events WHERE event_id=%s", (event_id,))
            event_pk = cur.fetchone()[0]

            # 3) vitals_samples（HR/RR 时序）
            cur.executemany(
                """
                INSERT IGNORE INTO vitals_samples (event_pk, t_s, hr_bpm, rr_bpm)
                VALUES (%s, %s, %s, %s)
                """,
                [(event_pk, t_s, hr, rr) for (t_s, hr, rr) in VITALS],
            )

            # 4) temp_samples（体温时序）
            cur.executemany(
                """
                INSERT IGNORE INTO temp_samples (event_pk, t_s, temp_c)
                VALUES (%s, %s, %s)
                """,
                [(event_pk, t_s, temp) for (t_s, temp) in TEMPS],
            )

            # 5) daily_reports（供 sql.search 演示）
            cur.execute(
                """
                INSERT INTO daily_reports (report_date, animal_id, risk_level, confidence, report_text)
                VALUES (CURDATE(), %s, %s, %s, %s)
                ON DUPLICATE KEY UPDATE risk_level=VALUES(risk_level),
                    confidence=VALUES(confidence), report_text=VALUES(report_text)
                """,
                (animal_id, 1, "medium", "联调种子日报：今日活动量正常，静息心率与体温在参考范围内，未见异常。"),
            )

        print(f"OK animal_id={animal_id} species={species} event_pk={event_pk} "
              f"vitals={len(VITALS)} temps={len(TEMPS)}")
    finally:
        conn.close()


if __name__ == "__main__":
    main()
