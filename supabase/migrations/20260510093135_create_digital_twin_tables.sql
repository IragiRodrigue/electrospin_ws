/*
  # Create Digital Twin Recording Tables

  1. New Tables
    - `twin_telemetry`
      - `id` (uuid, primary key)
      - `recorded_at` (timestamptz)
      - `pump_flow_ml_hr` (float)
      - `pump_pressure_kpa` (float)
      - `pump_running` (boolean)
      - `pump_tubing_wear` (float)
      - `pump_roller_wear` (float)
      - `pump_motor_temp_c` (float)
      - `collector_rpm` (float)
      - `collector_vibration` (float)
      - `collector_temp_c` (float)
      - `collector_bearing_wear` (float)
      - `collector_belt_wear` (float)
      - `hv_voltage_kv` (float)
      - `hv_enabled` (boolean)
      - `hv_arc_count` (integer)
      - `hv_insulation_wear` (float)
      - `env_temp_c` (float)
      - `env_humidity_pct` (float)
      - `dep_coverage_pct` (float)
      - `dep_total_mg` (float)
      - `health_score` (float)

    - `twin_alerts`
      - `id` (uuid, primary key)
      - `recorded_at` (timestamptz)
      - `alert_text` (text)

  2. Security
    - Enable RLS on both tables
    - Allow anonymous insert (digital twin bridge writes data)
    - Allow authenticated read (dashboard reads data)
*/

CREATE TABLE IF NOT EXISTS twin_telemetry (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  recorded_at timestamptz NOT NULL DEFAULT now(),
  pump_flow_ml_hr float DEFAULT 0,
  pump_pressure_kpa float DEFAULT 0,
  pump_running boolean DEFAULT false,
  pump_tubing_wear float DEFAULT 0,
  pump_roller_wear float DEFAULT 0,
  pump_motor_temp_c float DEFAULT 25,
  collector_rpm float DEFAULT 0,
  collector_vibration float DEFAULT 0,
  collector_temp_c float DEFAULT 25,
  collector_bearing_wear float DEFAULT 0,
  collector_belt_wear float DEFAULT 0,
  hv_voltage_kv float DEFAULT 0,
  hv_enabled boolean DEFAULT false,
  hv_arc_count integer DEFAULT 0,
  hv_insulation_wear float DEFAULT 0,
  env_temp_c float DEFAULT 22,
  env_humidity_pct float DEFAULT 45,
  dep_coverage_pct float DEFAULT 0,
  dep_total_mg float DEFAULT 0,
  health_score float DEFAULT 1.0
);

ALTER TABLE twin_telemetry ENABLE ROW LEVEL SECURITY;

CREATE POLICY "Allow anonymous insert for digital twin bridge"
  ON twin_telemetry FOR INSERT
  TO anon, authenticated
  WITH CHECK (true);

CREATE POLICY "Allow authenticated read of telemetry"
  ON twin_telemetry FOR SELECT
  TO authenticated
  USING (true);

CREATE POLICY "Allow anonymous read of telemetry"
  ON twin_telemetry FOR SELECT
  TO anon
  USING (true);

CREATE TABLE IF NOT EXISTS twin_alerts (
  id uuid PRIMARY KEY DEFAULT gen_random_uuid(),
  recorded_at timestamptz NOT NULL DEFAULT now(),
  alert_text text NOT NULL
);

ALTER TABLE twin_alerts ENABLE ROW LEVEL SECURITY;

CREATE POLICY "Allow anonymous insert for alerts"
  ON twin_alerts FOR INSERT
  TO anon, authenticated
  WITH CHECK (true);

CREATE POLICY "Allow authenticated read of alerts"
  ON twin_alerts FOR SELECT
  TO authenticated
  USING (true);

CREATE POLICY "Allow anonymous read of alerts"
  ON twin_alerts FOR SELECT
  TO anon
  USING (true);

-- Index for time-range queries
CREATE INDEX IF NOT EXISTS idx_telemetry_recorded_at ON twin_telemetry (recorded_at DESC);
CREATE INDEX IF NOT EXISTS idx_alerts_recorded_at ON twin_alerts (recorded_at DESC);
