namespace MagicMotion
{
    public struct MMGlobalData
    {
        public bool isContinue=>leastLoopCount!=0;
        public int leastLoopCount;
        public bool isInitialize;
    }
}