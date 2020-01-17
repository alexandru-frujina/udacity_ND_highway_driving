#define SAFE_DISTANCE 30.0
#define SAFE_LANE_CHANGE_GAP 0.0
#define FRONT_SPEED_RANGE 200.0

int laneFromD(float d)
{
  int lane = 0;
  
  if (d >= 0 && d < 4)
  {
    lane = 0;
  }
  else if (d >= 4 && d < 8)
  {
    lane = 1;
  }
  else if (d >= 8 && d < 12)
  {
    lane = 2;
  }
  
  return lane;
}

int getFastestLane(vector<vector<double>> sensor_fusion, double s_min, double s_max)
{
  int fastest_lane = 0;
  double fastest_lane_speed = 0.0;
  vector<double> lane_speeds = {49.5, 49.5, 49.5};
  
  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    int lane;
    float d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double v = sqrt(vx*vx + vy*vy);
    double s = sensor_fusion[i][5];
    
    if (s < s_min || s > s_max)
    {
      continue;
    }
    
    lane = laneFromD(d);
    
    if (lane_speeds[lane] > v)
    {
      lane_speeds[lane] = v;
    }
  }
  
  // Prioritize on the right lane
  fastest_lane = 2;
  fastest_lane_speed = lane_speeds[fastest_lane];
  
  for (int i = lane_speeds.size() - 1; i >= 0; i--)
  {
    if (fastest_lane_speed < lane_speeds[i])
    {
      fastest_lane_speed = lane_speeds[i];
      fastest_lane = i;
    }
  }
  
  return fastest_lane;
}

double getLaneChangeSafety(vector<vector<double>> sensor_fusion, int current_lane,
                            int target_lane, double ego_s, double ego_v, int prev_size)
{
  double safety = 1.0;
  int next_lane = 0;
  
  if (target_lane == current_lane)
  {
    return safety;
  }
  else if (target_lane > current_lane)
  {
    next_lane = current_lane + 1;
  }
  else
  {
    next_lane = current_lane - 1;
  }
  
  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double car_v = sqrt(vx*vx + vy*vy);
    double car_s = sensor_fusion[i][5];
    float d = sensor_fusion[i][6];
    
    if (next_lane == laneFromD(d))
    {
      double car_s_next = car_s + ((double)15.0 * 0.02 * car_v);
      double ego_s_next = ego_s + ((double)prev_size * 0.02 * ego_v);
      
      if ((ego_s > car_s - SAFE_LANE_CHANGE_GAP || ego_s_next > car_s - SAFE_LANE_CHANGE_GAP) 
        && (ego_s < car_s_next + SAFE_LANE_CHANGE_GAP || ego_s_next < car_s_next + SAFE_LANE_CHANGE_GAP))
      {
        safety = 0.0;
        break;
      }
    }
  }
  
  return safety;
}

double getFrontVehicleSpeed(vector<vector<double>> sensor_fusion, double ego_s, int ego_lane)
{
  double front_vehicle_speed = 49.5;
  double closest_s = ego_s + FRONT_SPEED_RANGE;
  
  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    int lane;
    float d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double v = sqrt(vx*vx + vy*vy);
    double s = sensor_fusion[i][5];
    
    if (ego_lane == laneFromD(d) && closest_s > s && s > ego_s)
    {
      closest_s = s;
      front_vehicle_speed = v;
    }
  }
  
  return front_vehicle_speed;
}