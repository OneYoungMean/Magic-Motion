namespace MagicMotion
{
    public struct GlobalData
    {
        public bool isContinue=>leastLoopCount!=0;
        public int axisDirection => leastLoopCount % 2 == 0 ? -1 : 1;
        public int leastLoopCount;
        public bool isInitialize;
    }
}