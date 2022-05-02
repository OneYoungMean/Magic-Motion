namespace MagicMotion
{
    public struct GlobalData
    {
        public bool isContinue=>leastLoopCount!=0;
        public int leastLoopCount;
        public bool isInitialize;
    }
}