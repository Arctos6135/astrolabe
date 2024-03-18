package astrolabe.follow;
  
import java.io.FileReader;

import org.json.simple.JSONArray; 
import org.json.simple.JSONObject; 
import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class AutoParser {
    public static AstrolabeAuto loadAuto(String pathName) throws Exception {
        JSONObject json = (JSONObject) new JSONParser().parse(new FileReader(pathName));
        Command commands = Commands.none();

        JSONArray comandMap = (JSONArray) ((JSONObject) ((JSONObject) json.get("command")).get("data")).get("commands");

        for (Object object : comandMap) {
            JSONObject command = (JSONObject) object;

            if (command.get("type").equals("path")) {
                commands = commands.andThen(new FollowPath("src/main/deploy/pathplanner/paths/" + ((JSONObject) command.get("data")).get("name") + ".path"));
            }

            else if (command.get("type").equals("named")) {
                commands = commands.andThen(NamedCommands.getCommand((String) ((JSONObject) command.get("data")).get("name")));
            }
        }

        return new AstrolabeAuto(commands);
    }
}
