-- PetMind schema reference for the agent's data tools.
--
-- sql.search WHITELIST (read-only SELECT, each auto-scoped to the request animal_id):
--     daily_reports, animals, sensor_events
--
-- vitals.summary reads the per-second TIME-SERIES tables below (NOT exposed to sql.search,
-- they have no animal_id column): vitals_samples, temp_samples, accel_samples.
-- They join back to a pet via: sample.event_pk -> sensor_events.id -> sensor_events.animal_id.
--
-- `devices` is referenced by sensor_events but is not whitelisted for the agent.

-- ============================== sql.search whitelist ==============================

CREATE TABLE IF NOT EXISTS daily_reports (
  id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  report_date DATE NOT NULL,
  animal_id VARCHAR(64) NOT NULL,
  risk_level TINYINT NULL,
  confidence ENUM('low','medium','high') NULL,
  report_text MEDIUMTEXT NOT NULL,
  report_json JSON NULL,
  evidence_json JSON NULL,
  agent_trace_id VARCHAR(64) NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (id),
  UNIQUE KEY uk_daily_reports_date_animal (report_date, animal_id),
  KEY idx_daily_reports_animal_date (animal_id, report_date)
) ENGINE=InnoDB;

-- Pet profile. species drives the MoE species soft-filter.
CREATE TABLE IF NOT EXISTS animals (
  id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  animal_id VARCHAR(64) NOT NULL,
  species ENUM('dog','cat','pig','sheep','cattle','horse','other') NOT NULL,
  name VARCHAR(64) NULL,
  breed VARCHAR(128) NULL,
  sex ENUM('male','female','unknown') NOT NULL DEFAULT 'unknown',
  age_months INT NULL,
  weight_kg DECIMAL(6,2) NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (id),
  UNIQUE KEY uk_animals_animal_id (animal_id),
  KEY idx_animals_species (species)
) ENGINE=InnoDB;

-- Collar upload window. `raw_payload` is excluded from the sql.search column whitelist.
CREATE TABLE IF NOT EXISTS sensor_events (
  id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  event_id VARCHAR(64) NOT NULL,
  ts DATETIME(3) NOT NULL,
  timezone VARCHAR(8) NULL,
  animal_id VARCHAR(64) NOT NULL,
  device_id VARCHAR(64) NULL,
  window_start DATETIME(3) NULL,
  window_end DATETIME(3) NULL,
  notes TEXT NULL,
  tags JSON NULL,
  location_lat DECIMAL(9,6) NULL,
  location_lng DECIMAL(9,6) NULL,
  location_accuracy_m DECIMAL(8,2) NULL,
  raw_payload JSON NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (id),
  UNIQUE KEY uk_sensor_events_event_id (event_id),
  KEY idx_sensor_events_animal_ts (animal_id, ts),
  KEY idx_sensor_events_device_ts (device_id, ts),
  CONSTRAINT fk_events_animal FOREIGN KEY (animal_id) REFERENCES animals (animal_id),
  CONSTRAINT fk_events_device FOREIGN KEY (device_id) REFERENCES devices (device_id)
) ENGINE=InnoDB;

-- ===================== time-series (vitals.summary only) =====================

CREATE TABLE IF NOT EXISTS vitals_samples (
  id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  event_pk BIGINT UNSIGNED NOT NULL,
  t_s INT UNSIGNED NOT NULL,
  hr_bpm SMALLINT UNSIGNED NULL,
  rr_bpm SMALLINT UNSIGNED NULL,
  PRIMARY KEY (id),
  UNIQUE KEY uk_vitals_event_t (event_pk, t_s),
  KEY idx_vitals_event (event_pk),
  CONSTRAINT fk_vitals_event FOREIGN KEY (event_pk) REFERENCES sensor_events (id) ON DELETE CASCADE
) ENGINE=InnoDB;

CREATE TABLE IF NOT EXISTS temp_samples (
  id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  event_pk BIGINT UNSIGNED NOT NULL,
  t_s INT UNSIGNED NOT NULL,
  temp_c DECIMAL(5,2) NOT NULL,
  PRIMARY KEY (id),
  UNIQUE KEY uk_temp_event_t (event_pk, t_s),
  KEY idx_temp_event (event_pk),
  CONSTRAINT fk_temp_event FOREIGN KEY (event_pk) REFERENCES sensor_events (id) ON DELETE CASCADE
) ENGINE=InnoDB;

CREATE TABLE IF NOT EXISTS accel_samples (
  id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  event_pk BIGINT UNSIGNED NOT NULL,
  t_ms INT UNSIGNED NOT NULL,
  x DECIMAL(10,4) NOT NULL,
  y DECIMAL(10,4) NOT NULL,
  z DECIMAL(10,4) NOT NULL,
  PRIMARY KEY (id),
  UNIQUE KEY uk_accel_event_t (event_pk, t_ms),
  KEY idx_accel_event (event_pk),
  CONSTRAINT fk_accel_event FOREIGN KEY (event_pk) REFERENCES sensor_events (id) ON DELETE CASCADE
) ENGINE=InnoDB;

-- ============================== referenced only ==============================

CREATE TABLE IF NOT EXISTS devices (
  id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT,
  device_id VARCHAR(64) NOT NULL,
  firmware VARCHAR(64) NULL,
  sampling_hz JSON NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (id),
  UNIQUE KEY uk_devices_device_id (device_id)
) ENGINE=InnoDB;
