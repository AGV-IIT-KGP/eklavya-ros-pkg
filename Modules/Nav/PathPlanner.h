namespace Nav
{
    class NavCore
    {
    public:
                static int mode;
                static void loadNavigator();
                static void navigate(char **map, CvPoint target);
                static void navigate(char **map, CvPoint target, char *logFileId, FILE *logFile, int frameCount);
                static void navigate(char **map, CvPoint target, double yaw);
                static void closeNav();
    };
}
