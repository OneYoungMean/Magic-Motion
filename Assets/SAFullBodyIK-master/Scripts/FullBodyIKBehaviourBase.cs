// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.php

using UnityEngine;
using System.Collections.Generic;

namespace SA
{
	public abstract class FullBodyIKBehaviourBase : MonoBehaviour
	{
		[System.NonSerialized]
		FullBodyIK _cache_fullBodyIK; // instance cache

		public abstract FullBodyIK fullBodyIK
		{
			get;
		}

        // Excecutable in Inspector.
        public virtual void Prefix()//OYM：固定啥的吧
        {
			if( _cache_fullBodyIK == null ) {
				_cache_fullBodyIK = fullBodyIK;
			}
			if( _cache_fullBodyIK != null ) {
				_cache_fullBodyIK.Prefix( this.transform );
			}
		}

		protected virtual void Awake()
		{
#if UNITY_EDITOR
			if( !Application.isPlaying ) {
				return;
			}
#endif
			if( _cache_fullBodyIK == null ) {
				_cache_fullBodyIK = fullBodyIK;
			}
			if( _cache_fullBodyIK != null ) {
				_cache_fullBodyIK.Awake( this.transform );
			}
		}

		protected virtual void OnDestroy()
		{
			if( _cache_fullBodyIK == null ) {
				_cache_fullBodyIK = fullBodyIK;
			}
			if( _cache_fullBodyIK != null ) {
                _cache_fullBodyIK.Destroy();//OYM：销毁
            }
		}

		protected virtual void LateUpdate()
		{
#if UNITY_EDITOR
			if( !Application.isPlaying ) {
				return;
			}
#endif
			if( _cache_fullBodyIK == null ) {
				_cache_fullBodyIK = fullBodyIK;
			}
			if( _cache_fullBodyIK != null ) {
                _cache_fullBodyIK.Update();//OYM：
            }
		}

#if UNITY_EDITOR
		void OnDrawGizmos()
		{
			if( _cache_fullBodyIK == null ) {
				_cache_fullBodyIK = fullBodyIK;
			}
			if( _cache_fullBodyIK != null ) {
				_cache_fullBodyIK.DrawGizmos();
			}
		}
#endif
	}
}