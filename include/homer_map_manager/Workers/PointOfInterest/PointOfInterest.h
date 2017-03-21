#ifndef POINTOFINTEREST_H
#define POINTOFINTEREST_H

#include <map>
#include <string>

#include <homer_nav_libs/Math/Pose.h>

using namespace std;

typedef std::map<std::string, std::string> StringMapT;
typedef std::map<std::string, float> FloatMapT;
typedef std::map<std::string, int> IntMapT;

/**
 * @class PointOfInterest
 *
 * @author Robert Hoffmann (RX), David Gossow (RX), Simon Graeser (RX),
 *         Nicolai Wojke (R14)
 *
 * @brief This class represents a point of interest (POI)
 *
 * This class represents a point of interest (POI). It is derived from
 * Point2D and thus inherits its methods x(), y(), and theta() to query
 * the world position and orientation.
 */

class PointOfInterest : public Pose
{
public:
  enum PoiType
  {
    DEFAULT = 100,
    VICTIM = 200,
    OBJECT = 300,
    GRIPPABLE_OBJECT = 400,
    PERSON = 600,
    ROOMBA = 700,
    HAZARD_MATERIAL = 800,
    START_POSITION = 900,
    START_ORIENTATION = 1000
  };

  PointOfInterest();

  /**
    * @brief The constructor
    *
    * Creates a point of interest with orientation 0.0.
    *
    * @param id unique identification number
    * @param name name of the poi
    * @param type type of the poi (victim, etc.)
    * @param posX,posY position within the map
    * @param remarks additional information associated with the poi
    */
  PointOfInterest(int id, string name, PoiType type, float posX, float posY,
                  string remarks, StringMapT stringMap = StringMapT(),
                  FloatMapT floatMap = FloatMapT(), IntMapT intMap = IntMapT())
    : Pose(posX, posY, 0.0f)
  {
    init(id, name, type, remarks, stringMap, floatMap, intMap);
  }

  /** @brief The constructor
    * @param id unique identification number
    * @param name name of the poi
    * @param type type of the poi (victim, etc.)
    * @param posX,posY position within the map
    * @param theta orientation of the poi
    * @param remarks additional information associated with the poi
    */
  PointOfInterest(int id, string name, PoiType type, float posX, float posY,
                  float theta, string remarks,
                  StringMapT stringMap = StringMapT(),
                  FloatMapT floatMap = FloatMapT(), IntMapT intMap = IntMapT())
    : Pose(posX, posY, theta)
  {
    init(id, name, type, remarks, stringMap, floatMap, intMap);
  }

  /** @brief Alternative constructor omitting the id parameter */
  PointOfInterest(string name, PoiType type, float posX, float posY,
                  string remarks)
    : Pose(posX, posY, 0.0f)
  {
    init(-1, name, type, remarks, StringMapT(), FloatMapT(), IntMapT());
  }

  /** @brief Alternative constructor omitting the id parameter */
  PointOfInterest(string name, PoiType type, float posX, float posY,
                  float theta, string remarks)
    : Pose(posX, posY, theta)
  {
    init(-1, name, type, remarks, StringMapT(), FloatMapT(), IntMapT());
  }

  /** @brief Copy constructor, assigns a new id */
  PointOfInterest(int id, const PointOfInterest* poi);

  /** @brief The standard destructor */
  virtual ~PointOfInterest()
  {
  }

  /** @brief Tests whether this PointOfInterest has the specified name or not,
   * ignoring case.
    * @param name The name to test as a string.
    * @return true if the given name matches the PointOfInterest's name (not
   * case-sensitive), false if not.
    */
  bool hasName(string name) const;

  /** @brief Tests whether this PointOfInterest has the specified string in its
   * name or not, ignoring case.
   * @param part The string to test.
   * @return true if the given string is contained in the PointOfInterest's name
   * (not case-sensitive), false if not.
   */
  bool hasInName(string part) const;

  // SETTER FUNCTIONS /////////////////////////////

  /** @param remarks Changes the contents of remarks attribute. */
  void setRemarks(string remarks)
  {
    m_Remarks = remarks;
  }

  void setName(string name)
  {
    m_Name = name;
  }

  /** @brief Add a new String to the StringMap */
  void addString(string key, string data)
  {
    m_StringMap.insert(pair<string, string>(key, data));
  }

  /** @brief Add a new Float to the FloatMap */
  void addFloat(string key, float data)
  {
    m_FloatMap.insert(pair<string, float>(key, data));
  }

  /** @brief Add a new Int to the IntMap */
  void addInt(string key, int data)
  {
    m_IntMap.insert(pair<string, int>(key, data));
  }

  // GETTER FUNCTIONS //////////////////////

  /** @return id attribute */
  int getId() const
  {
    return m_Id;
  }

  /** @return name attribute */
  string getName() const
  {
    return m_Name;
  }

  /** @return type attribute */
  PoiType getType() const
  {
    return m_Type;
  }

  /** @return remarks attribute */
  string getRemarks() const
  {
    return m_Remarks;
  }

  /** Get the string to the matching string */
  string getString(string key) const
  {
    return m_StringMap.find(key)->second;
  }

  /** Get the float to the matching string */
  float getFloat(string key) const
  {
    return m_FloatMap.find(key)->second;
  }

  /** Get the Int to the matching string */
  int getInt(string key) const
  {
    return m_IntMap.find(key)->second;
  }

  /** Get the string map */
  map<string, string> getStringMap() const
  {
    return m_StringMap;
  }

  /** Get the float map */
  map<string, float> getFloatMap() const
  {
    return m_FloatMap;
  }

  /** Get the int map */
  map<string, int> getIntMap() const
  {
    return m_IntMap;
  }

  // (DE)SERIALIZATION ///////////////////////////////////
  // TODO
  //    void storer ( ExtendedOutStream& ) const ;
  //    PointOfInterest ( ExtendedInStream& );

  /** @brief print description */
  void printOn(ostream& strm) const;

private:
  void init(int id, string name, PoiType type, string remarks,
            StringMapT stringMap, FloatMapT floatMap, IntMapT intMap);

  int m_Id;
  string m_Name;
  PoiType m_Type;
  string m_Remarks;

  StringMapT m_StringMap;
  IntMapT m_IntMap;
  FloatMapT m_FloatMap;
};

#endif
