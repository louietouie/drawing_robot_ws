


// the pseudoinverse controller will need the URDF to calculate the inverse jacobian (aka it needs to know the robot dimensions/joint locations).
parser.AddModelFromFile()
parser.AddModelFromUrl()
parser.AddModelFromString() // give it the raw string """<robot>...</robot>"""\

// the controller has access to the post-xacro-parsed URDF (it was given in the launch file)
// {'robot_description': robot_description_config.toxml()}
// how do I access this?

const std::string & urdf = get_robot_description();
if (!urdf.empty()) {
    parser.AddModelFromString(urdf);
}