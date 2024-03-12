package astrolabe;
  
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Iterator; 
import java.util.Map; 
  
import org.json.simple.JSONArray; 
import org.json.simple.JSONObject; 
import org.json.simple.parser.*;
import org.json.simple.parser.JSONParser; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;



public class PathParser {
    public AstrolabePath astrolabePath;

    public void LoadPath(String pathName) throws Exception{
        Object obj = new JSONParser().parse(new FileReader(pathName));           
        // typecasting obj to JSONObject 
        JSONObject jo = (JSONObject) obj;

        JSONArray wayPoints = (JSONArray) jo.get("waypoints");
        
        // iterating phoneNumbers 
        Map<String, Map<String, Double>> itr1;
        Iterator itr2 = wayPoints.iterator(); 
        List<Pose2d> poses = new ArrayList<>();
        
        while (itr2.hasNext()) { 
            itr1 = ((Map) itr2.next()); 
            double x = itr1.get("anchor").get("x"); 
            double y = itr1.get("anchor").get("y"); 
            double rotationX = itr1.get("nextControl").get("x"); 
            double rotationY = itr1.get("nextControl").get("y"); 
            poses.add(new Pose2d(x, y, new Rotation2d(rotationX, rotationY)));
        }
        thisastrolabePath = new AstrolabePath(poses.get(0), new ArrayList<Translation2d>(), poses.get(1), (boolean) jo.get("reversed"))
    }
}