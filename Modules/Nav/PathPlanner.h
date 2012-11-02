namespace Nav
{
  class NavCore
  {
    public:
      static int mode;
      static void loadNavigator();
      static int navigate(char **map, CvPoint target, int frameCount, double yaw);
      static void closeNav();
  };
}
