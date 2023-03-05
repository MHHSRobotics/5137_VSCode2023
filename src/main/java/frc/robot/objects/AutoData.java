package frc.robot.objects;

public class AutoData {
    private String pos;
    private String type;
    private Boolean mobility;
    private Boolean engage;

     /**
     * AutoData groups the information in auto for transport across systems.
     * Middle Position does not accept mobility input or DoubleScoring
     * DoubleScoring nullifies mobility input.
     * @param kPos Position of the bot. Accepts Left, Middle, or Right.
     * @param kType The type of auto. Accepts None, SingleScore, or DoubleScore
     * @param kMobility Auto with mobility
     * @param kEngage Auto with charge station engagement.
    */
    public AutoData(String kPos, String kType, Boolean kMobility, Boolean kEngage) {
        this.pos = kPos;
        this.type = kType;
        this.engage = kEngage;
        if (pos == "Middle") {
            mobility = false;
            if (type == "DoubleScore") {
                type = "None";
            }
        } else {
            this.mobility = kMobility;
        }
        if (type == "DoubleScore") {
            mobility = true;
        }
    }

    public String getPosition() {
        return pos;
    }

    public String getType() {
        return type;
    }

    public Boolean getMobility() {
        return mobility;
    }

    public Boolean getEngage() {
        return engage;
    }

    public Boolean equalTo(AutoData x) {
        if (x.getPosition() == pos && x.getType() == type && x.getMobility() == mobility && x.getEngage() == engage) {
            return true;
        } else {
            return false;
        }
    }
}
