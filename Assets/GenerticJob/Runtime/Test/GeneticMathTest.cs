using GeneticJob;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static GeneticJob.GeneticMath;
public class GeneticMathTest : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        DoMathTest();
        DoThrowTest();
    }


    void DoMathTest()
    {
        Debug.Log(ByteTofloat(0xff)+" ==1");
        Debug.Log(ByteTofloat(0x80)+"==0.5");
        Debug.Log(ByteTofloat(0x01) + "==0.004");

        Debug.Log(UshortTofloat(0xffff) + " ==1");
        Debug.Log(UshortTofloat(0x8000) + "==0.5");
        Debug.Log(UshortTofloat(0x0001) + "==6e-5");

        Debug.Log(UintTofloat(0xffffffff) + " ==1");
        Debug.Log(UintTofloat(0x80000000) + "==0.5");
        Debug.Log(UintTofloat(0x00000001) + "==2e-10");

        Bool32 bool32 = new Bool32();
        bool32[31] = true;
        bool32[0] = true;

        Debug.Log(UintTofloat3(0xffffffff).ToString()+" ==1,1,1");
        Debug.Log(UintTofloat3(0x00000000).ToString() + " ==0,0,0");
        Debug.Log(Float3ToUint( UintTofloat3(12345678u)).ToString()+" ==12345678");

        Debug.Log(UintTofloat4(0xffffffff).ToString() + " ==1,1,1.1");
        Debug.Log(UintTofloat4(0x00000000).ToString() + " ==0,0,0,0");
        Debug.Log(Float4ToUint(UintTofloat4(12345678u)).ToString() + " ==12345678");

;
        Debug.Log(Float3ToUlong(UlongTofloat3(0xffff_ffff_ffff_ffff)).ToString() + " =="+ulong.MaxValue);

        Debug.Log(Float4ToUlong(UlongTofloat4(0xffff_ffff_ffff_ffff)).ToString() + " ==" + ulong.MaxValue);

        Debug.Log(UintToBinary( Mutation_FilpBit(0,16)));
        Debug.Log(UintToBinary(Mutation_FilpBitFast(0)));
        Debug.Log(UintToBinary(Mutation_FilpUint(0)));
        Debug.Log(UintToBinary(Mutation_ReverseUint(0xAAAAAAAA)));
        Debug.Log(UintToBinary(Mutation_FastMutationUint(0xAAAAAAAA)));
        Debug.Log(UintToBinary(Mutation_DisplacementUint(0x00AAAA00,16)));
    }

    void DoThrowTest()
    {

    }
}
