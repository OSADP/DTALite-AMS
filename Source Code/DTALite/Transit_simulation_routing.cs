1. Finding shortest path
public List<DynamicVertex> LabelCorrecting(Network net, double origin_x, double origin_y,
            double D_transfer, double Transfer_unit_time, int depart_time, int transfer_max_count, int T_min, int T_max, Dictionary<string, int> visiting_dynamic_vertex_map)
        {
            List<DynamicVertex> dvList = new List<DynamicVertex>();
            visiting_dynamic_vertex_map = new Dictionary<string, int>();
            Dictionary<string, int> current_dynamic_vertex_map = new Dictionary<string, int>();
            List<DynamicVertex> visting_dynamic_vertex_list = new List<DynamicVertex>();

            DynamicVertex dv = new DynamicVertex();
            //1 origin vertex
            dv.Stop = -1; dv.Time = depart_time; dv.Vehicle = -1; dv.N = 0; dv.LabelCost = 0;
            //dvList.Add(dv);
            visting_dynamic_vertex_list.Add(dv);
            visiting_dynamic_vertex_map.Add("-1;" + depart_time.ToString() + ";-1;0", visiting_dynamic_vertex_map.Count);

            int now_x, now_y, now_t;
            //1.1 add first stop to list
            #region find neighbour
            now_x = (int)((origin_x - min_x) / unit_grid_distance);
            now_y = (int)((origin_y - min_y) / unit_grid_distance);
            now_t = depart_time;

            if (now_x >= GridLength)
                now_x = GridLength - 1;
            if (now_y >= GridWidth)
                now_y = GridWidth - 1;

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    for (int t = T_min; t <= T_max; t++)
                    {
                        int new_x, new_y, new_t;
                        new_x = now_x + x;
                        new_y = now_y + y;
                        new_t = now_t + t;

                        if (new_x >= 0 && new_x < GridLength && new_y >= 0 && new_y < GridWidth && new_t >= 0 && new_t <= net.endTimeInterval)
                        {
                            if (GridList[new_x, new_y, new_t] != null && GridList[new_x, new_y, new_t].VertexList.Count > 0)
                            {
                                foreach (Vertex new_vertex in GridList[new_x, new_y, new_t].VertexList)
                                {
                                    //2.1 check the distance
                                    double node_x, node_y;
                                    node_x = net.TransitNodeList[new_vertex.Stop].X;
                                    node_y = net.TransitNodeList[new_vertex.Stop].Y;

                                    double length = (new Vector(node_x, node_y) - new Vector(origin_x, origin_y)).Len();
                                    if (length < D_transfer)
                                    {
                                        //2.2 check the transfer time
                                        double transfer_time = length * Transfer_unit_time;
                                        if (new_t - now_t >= transfer_time)
                                        {
                                            string vertex_string;
                                            vertex_string = new_vertex.Stop.ToString() + ";" +
                                                new_vertex.Time.ToString() + ";" +
                                                new_vertex.Vehicle.ToString() + ";0";

                                            if (!visiting_dynamic_vertex_map.ContainsKey(vertex_string))
                                            {
                                                DynamicVertex dv_new = new DynamicVertex();
                                                dv_new.Stop = new_vertex.Stop;
                                                dv_new.Time = new_vertex.Time;
                                                dv_new.Vehicle = new_vertex.Vehicle;
                                                dv_new.N = 0;
                                                dv_new.ForeArcList = new_vertex.ForeArcList;
                                                dv_new.LabelCost = t;
                                                dv_new.PreDynamicVertex = dv;

                                                visiting_dynamic_vertex_map.Add(vertex_string, visiting_dynamic_vertex_map.Count);
                                                visting_dynamic_vertex_list.Add(dv_new);
                                            }

                                            if (!current_dynamic_vertex_map.ContainsKey(vertex_string))
                                            {
                                                dvList.Add(visting_dynamic_vertex_list[visiting_dynamic_vertex_map[vertex_string]]);
                                                current_dynamic_vertex_map.Add(vertex_string, visiting_dynamic_vertex_map.Count);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            #endregion

            //2 label correcting
            while (dvList.Count > 0)
            {
                DynamicVertex now_vertex = dvList[0];

                #region check this vertex is visited
                string temp_now_vertex_string = now_vertex.Stop.ToString() + ";" +
                            now_vertex.Time.ToString() + ";" +
                            now_vertex.Vehicle.ToString();
                Vertex temp_now_vertex = VertexList[VertexMap[temp_now_vertex_string]];
                if (!temp_now_vertex.IsVisited)
                {
                    CreateTransferArcForOneVertex(net, depart_time, 120, temp_now_vertex, T_min, T_max, D_transfer, Transfer_unit_time);
                    temp_now_vertex.IsVisited = true;
                }
                else
                {
                    int aaa = -1;
                }
                #endregion

                int now_transfer_count = now_vertex.N;
                int now_vehicle = now_vertex.Vehicle;

                string now_vertex_string = now_vertex.Stop.ToString() + ";" +
                            now_vertex.Time.ToString() + ";" +
                            now_vertex.Vehicle.ToString() + ";" +
                            now_vertex.N.ToString();

                //2.1 foreward searching
                #region foreward searching
                foreach (Arc arc in now_vertex.ForeArcList)
                {
                    Vertex vertex = arc.ToVertex;
                    int vehicle = vertex.Vehicle;

                    int new_transfer_count = now_transfer_count;
                    if (vehicle != now_vehicle)
                        new_transfer_count++;

                    //2.2 check the max transfer time constraint
                    if (new_transfer_count <= transfer_max_count)
                    {
                        string vertex_string;
                        vertex_string = vertex.Stop.ToString() + ";" +
                            vertex.Time.ToString() + ";" +
                            vertex.Vehicle.ToString() + ";" +
                            new_transfer_count.ToString();

                        if (!visiting_dynamic_vertex_map.ContainsKey(vertex_string))
                        {
                            DynamicVertex dv_new = new DynamicVertex();
                            dv_new.Stop = vertex.Stop;
                            dv_new.Time = vertex.Time;
                            dv_new.Vehicle = vertex.Vehicle;
                            dv_new.N = new_transfer_count;
                            dv_new.ForeArcList = vertex.ForeArcList;
                            dv_new.LabelCost = 999999;

                            visiting_dynamic_vertex_map.Add(vertex_string, visiting_dynamic_vertex_map.Count);
                            visting_dynamic_vertex_list.Add(dv_new);
                        }

                        //2.3 label correcting
                        if (!current_dynamic_vertex_map.ContainsKey(vertex_string))
                        {
                            dvList.Add(visting_dynamic_vertex_list[visiting_dynamic_vertex_map[vertex_string]]);
                            current_dynamic_vertex_map.Add(vertex_string, 0);
                        }

                        DynamicVertex dv_temp = visting_dynamic_vertex_list[visiting_dynamic_vertex_map[vertex_string]];
                        if (now_vertex.LabelCost + arc.Cost < dv_temp.LabelCost)
                        {
                            dv_temp.LabelCost = now_vertex.LabelCost + arc.Cost;
                            dv_temp.PreDynamicVertex = now_vertex;
                        }
                    }
                }
                #endregion

                dvList.RemoveAt(0);
                current_dynamic_vertex_map.Remove(now_vertex_string);
            }

            return visting_dynamic_vertex_list;
        }


public List<DynamicPath> FindShortestPath(Network net, List<DynamicVertex> dvList,
            double destination_x, double destination_y, int depart_time, int max_time, double r)
        {
            List<int> stop_list = new List<int>();
            Dictionary<int, int> stop_map = new Dictionary<int, int>();

            //1 find the stop list
            foreach (Node n in net.TransitNodeList)
            {
                double x, y;
                x = n.X; y = n.Y;

                double distance = (destination_x - x) * (destination_x - x) + (destination_y - y) * (destination_y - y);
                if (distance < r * r)
                {
                    stop_list.Add(n.Index);
                    stop_map.Add(n.Index, stop_map.Count);
                }
            }

            //2 find the shortest path
            float[] best_cost = new float[stop_list.Count];
            DynamicVertex[] best_dv = new DynamicVertex[stop_list.Count];

            for (int i = 0; i < stop_list.Count; i++)
            {
                best_cost[i] = 999999;
            }

            foreach (DynamicVertex dv in dvList)
            {
                int stop = dv.Stop;

                if (stop_map.ContainsKey(stop))
                {
                    int stop_index = stop_map[stop];

                    if (dv.LabelCost < best_cost[stop_index])
                    {
                        best_cost[stop_index] = dv.LabelCost;
                        best_dv[stop_index] = dv;
                    }
                }
            }

            //-new version of finding optimal path---//
            //for (int i = 0; i < stop_list.Count; i++)
            //{

            //}
            //---------------------------------------//

            ////3 output to file
            //StreamWriter sw = new StreamWriter("test_shortest_path.csv");
            //sw.WriteLine("destination_station,travel_time_in_min,path_node_sequence");
            
            //3 find all path
            List<DynamicPath> dpList = new List<DynamicPath>();

            for (int i = 0; i < stop_list.Count; i++)
            {
                if (best_dv[i] != null)
                {
                    DynamicPath dp = new DynamicPath();
                    dp.destination = best_dv[i].Stop;
                    dp.cost = best_dv[i].LabelCost;
                    dp.transfer_count = best_dv[i].N;
                    dp.DVList.Add(best_dv[i]);

                    DynamicVertex dv_now = best_dv[i];
                    while(dv_now.PreDynamicVertex!=null)
                    {
                        dp.DVList.Insert(0,dv_now.PreDynamicVertex);
                        dv_now = dv_now.PreDynamicVertex;
                    }

                    //reformualte the dvlist for output
                    #region reformulate the dvlist
                    //1 restore the dv list
                    List<int> temp_index = new List<int>();
                    int current_vehicle = -1;
                    for (int k = 0; k < dp.DVList.Count; k++)
                    {
                        int temp_v = dp.DVList[k].Vehicle;
                        if (temp_v > 0)
                        {
                            if (temp_v != current_vehicle)
                            {
                                temp_index.Add(k);
                                current_vehicle = temp_v;
                            }
                        }
                    }
                    temp_index.Add(dp.DVList.Count);

                    //1.1 insert vehicle
                    List<DynamicVertex> temp_dvlist = new List<DynamicVertex>();
                    for (int k = 0; k < temp_index.Count - 1; k++)
                    {
                        if (temp_index[k + 1] - temp_index[k] > 1)
                        {
                            temp_dvlist.Add(dp.DVList[temp_index[k]]);
                            temp_dvlist.Add(dp.DVList[temp_index[k + 1] - 1]);
                        }
                    }

                    dp.DVListForOutput = temp_dvlist;
                    //2 check is feasible
                    bool isFeasible = false;
                    if (temp_dvlist.Count > 0)
                        isFeasible = true;
                    dp.isFeasiblePath = isFeasible;
                    #endregion

                    if (dp.isFeasiblePath)
                        dpList.Add(dp);
                    #region for output
                    //string path_node_string = "";

                    //DynamicVertex dv_now = best_dv[i];
                    //string new_node = "";
                    //new_node = dv_now.Stop.ToString() + "|" +
                    //        dv_now.Time.ToString() + "|" +
                    //        dv_now.Vehicle.ToString() + "|" +
                    //        dv_now.N.ToString();
                    //path_node_string = new_node;

                    //while (dv_now.PreDynamicVertex != null)
                    //{
                    //    dv_now = dv_now.PreDynamicVertex;
                    //    new_node = dv_now.Stop.ToString() + "|" +
                    //       dv_now.Time.ToString() + "|" +
                    //       dv_now.Vehicle.ToString() + "|" +
                    //       dv_now.N.ToString();

                    //    path_node_string = new_node + ";" + path_node_string;
                    //}

                    //sw.WriteLine(net.TransitNodeList[stop_list[i]].NodeString + "," +
                    //    best_cost[i].ToString() + "," + path_node_string);
                    #endregion
                }
            }
            //sw.Close();
            return dpList;
        }


2. Read GTFS data
public void ReadGTFSNodeData(string transit_stop_file_name)
        {
            TransitNodeList.Clear();
            TransitNodeMap.Clear();

            #region read transit node
            CSVParser csv_node = new CSVParser(transit_stop_file_name);
            csv_node.Open();

            csv_node.ReadHeadTitle();
            while (!csv_node.IsEndOfStream())
            {
                if (csv_node.ReadDataByLine())
                {

                    Node n = new Node();
                    n.Index = TransitNodeList.Count;
                    csv_node.GetFieldValue("stop_id", out n.ID);
                    csv_node.GetFieldValue("stop_name", out n.NodeString);

                    csv_node.GetFieldValue("stop_lat", out n.Y);
                    csv_node.GetFieldValue("stop_lon", out n.X);

                    n.Y = -n.Y;
                    n.R = 0.00067;
                    //n.R = 0.028;

                    TransitNodeList.Add(n);
                    TransitNodeMap.Add(n.ID, n.Index);
                }

            }
            csv_node.Close();
            #endregion

        }

        public void ReadGTFSShapeData(string transit_shape_file_name)
        {
            TransitShapeList.Clear();
            TransitShapeMap.Clear();

            #region read transit node
            CSVParser csv = new CSVParser(transit_shape_file_name);
            csv.Open();

            csv.ReadHeadTitle();
            while (!csv.IsEndOfStream())
            {
                if (csv.ReadDataByLine())
                {
                    TransitLine ts = new TransitLine();

                    string shape_id;
                    csv.GetFieldValue("shape_id", out shape_id);

                    if (TransitShapeMap.ContainsKey(shape_id))
                    {
                        int shape_index = TransitShapeMap[shape_id];
                        ts = TransitShapeList[shape_index];
                    }
                    else
                    {
                        ts.ID = shape_id;
                        TransitShapeMap.Add(shape_id, TransitShapeMap.Count);

                        ts.Geometry = new List<ShapePoint>();
                        TransitShapeList.Add(ts);
                    }

                    double x, y;
                    int sequence;
                    csv.GetFieldValue("shape_pt_lat", out x);
                    csv.GetFieldValue("shape_pt_lon", out y);
                    csv.GetFieldValue("shape_pt_sequence", out sequence);

                    ShapePoint sp = new ShapePoint();
                    sp.Point = new Vector(y, -x);
                    sp.Sequence = sequence;

                    ts.Geometry.Add(sp);
                }
            }
            csv.Close();
            #endregion

            //12.15=======new add========find physical node=====//
            NodeList.Clear();
            NodeMap.Clear();
            LinkList.Clear();
            LinkMap.Clear();
            //==================================================//

            //arrange the sequence
            #region arrange the sequence
            foreach (TransitLine ts in TransitShapeList)
            {
                if (ts.ID == "1799")
                {
                    int ccdc = -1;
                }
                if (ts.Geometry.Count > 1)
                {
                    ts.Geometry = ts.Geometry.OrderBy(tss => tss.Sequence).ToList();

                    //12.15=======new add========find physical node=====//
                    #region find physical node
                    //find first node and last node

                    //first node
                    Node phy_node = new Node();
                    phy_node.Index = NodeList.Count;
                    phy_node.X = ts.Geometry[0].Point.x;
                    phy_node.Y = ts.Geometry[0].Point.y;
                    phy_node.R = 0.0024;
                    phy_node.Type = "station";

                    if (!IsNodeExisted(phy_node.X, phy_node.Y, NodeList))
                    {
                        NodeList.Add(phy_node);
                    }


                    //last node
                    phy_node = new Node();
                    phy_node.Index = NodeList.Count;
                    phy_node.X = ts.Geometry[ts.Geometry.Count - 1].Point.x;
                    phy_node.Y = ts.Geometry[ts.Geometry.Count - 1].Point.y;
                    phy_node.R = 0.0024;
                    phy_node.Type = "station";

                    if (!IsNodeExisted(phy_node.X, phy_node.Y, NodeList))
                    {
                        NodeList.Add(phy_node);
                    }
                    #endregion
                    //==================================================//

                    //remove same points
                    List<ShapePoint> spList = new List<ShapePoint>();
                    spList.Add(ts.Geometry[0]);

                    for (int i = 1; i < ts.Geometry.Count; i++)
                    {
                        double distance = (ts.Geometry[i].Point - ts.Geometry[i - 1].Point).Len();
                        if (distance > 0.000001)
                        //if(true)
                        {
                            spList.Add(ts.Geometry[i]);
                        }
                        else
                        {
                            //12.15=======new add========find physical node=====//
                            //find common node
                            #region find physical node
                            phy_node = new Node();
                            phy_node.Index = NodeList.Count;
                            phy_node.X = ts.Geometry[i].Point.x;
                            phy_node.Y = ts.Geometry[i].Point.y;
                            phy_node.R = 0.0024;
                            phy_node.Type = "station";

                            if (!IsNodeExisted(phy_node.X, phy_node.Y, NodeList))
                            {
                                NodeList.Add(phy_node);
                            }
                            #endregion
                            //==================================================//

                        }
                    }

                    ts.Geometry = spList;

                    //geometry
                    ts.CenterPoint = new Vector[ts.Geometry.Count];
                    for (int i = 0; i < ts.Geometry.Count; i++)
                    {
                        ts.CenterPoint[i] = ts.Geometry[i].Point;
                    }
                }
            }
            #endregion


           
        }

        public void ReadGTFStopTimeData(string transit_stop_time_file_name)
        {
            TransitVehicleList.Clear();
            TransitVehicleMap.Clear();     

            #region read transit node
            CSVParser csv = new CSVParser(transit_stop_time_file_name);
            csv.Open();

            csv.ReadHeadTitle();
            while (!csv.IsEndOfStream())
            {
                if (csv.ReadDataByLine())
                {
                    TransitVehicle ta = new TransitVehicle();
                    ta.Index = TransitVehicleList.Count;

                    string agent_id;
                    csv.GetFieldValue("trip_id", out agent_id);

                    if (TransitVehicleMap.ContainsKey(agent_id))
                    {
                        int agent_index = TransitVehicleMap[agent_id];
                        ta = TransitVehicleList[agent_index];
                    }
                    else
                    {
                        TransitVehicleMap.Add(agent_id, TransitVehicleMap.Count);
                        ta.ID = agent_id;
                        ta.STDList = new List<StopTimeData>();
                        TransitVehicleList.Add(ta);
                    }

                    string arr_time, dep_time;
                    string stop_id;
                    int sequence;

                    csv.GetFieldValue("arrival_time", out arr_time);
                    csv.GetFieldValue("departure_time", out dep_time);
                    csv.GetFieldValue("stop_id", out stop_id);
                    csv.GetFieldValue("stop_sequence", out sequence);

                    //string transit_line;
                    //csv.GetFieldValue("transit_line", out transit_line);
                    //ta.LineID = TransitShapeMap[transit_line];

                    //string capacity;
                    //csv.GetFieldValue("capacity", out capacity);

                    //if (capacity.Trim() != "")
                    //{
                    //    ta.CapPassenger = Convert.ToInt16(capacity);
                    //}

                    StopTimeData std = new StopTimeData();
                    std.StopTime = new int[2];
                    std.StopTime[0] = ConvertTimeStringToTimeInterval(arr_time);
                    std.StopTime[1] = ConvertTimeStringToTimeInterval(dep_time);
                    std.StopID = TransitNodeMap[stop_id];
                    std.Sequence = sequence;

                    ta.STDList.Add(std);
                }
            }

            csv.Close();
            #endregion

            #region order sequence
            foreach (TransitVehicle ta in TransitVehicleList)
            {
                ta.STDList = ta.STDList.OrderBy(std => std.Sequence).ToList();

                //convert stdlist to array

                ta.StopTime = new int[ta.STDList.Count, 2];
                ta.StopIDList = new int[ta.STDList.Count];

                for (int i = 0; i < ta.STDList.Count; i++)
                {
                    ta.StopTime[i, 0] = ta.STDList[i].StopTime[0];
                    ta.StopTime[i, 1] = ta.STDList[i].StopTime[1];
                    ta.StopIDList[i] = ta.STDList[i].StopID;
                }
            }
            #endregion
        }
