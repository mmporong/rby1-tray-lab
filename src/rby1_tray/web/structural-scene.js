(() => {
  'use strict';

  const fetchBuffer = async (url) => {
    const response = await fetch(url, {cache: 'no-store'});
    if (!response.ok) throw new Error(`응력 데이터 요청 실패 (${response.status})`);
    return response.arrayBuffer();
  };

  const compile = (gl, type, source) => {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source); gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      const message = gl.getShaderInfoLog(shader); gl.deleteShader(shader);
      throw new Error(`WebGL 셰이더 오류: ${message}`);
    }
    return shader;
  };

  const program = (gl, vertex, fragment) => {
    const result = gl.createProgram();
    gl.attachShader(result, compile(gl, gl.VERTEX_SHADER, vertex));
    gl.attachShader(result, compile(gl, gl.FRAGMENT_SHADER, fragment));
    gl.linkProgram(result);
    if (!gl.getProgramParameter(result, gl.LINK_STATUS)) throw new Error(`WebGL 연결 오류: ${gl.getProgramInfoLog(result)}`);
    return result;
  };

  const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const normalize = (v) => { const n=Math.hypot(...v)||1; return v.map((x)=>x/n); };
  const lookAt = (eye, center) => {
    const z=normalize(eye.map((x,i)=>x-center[i])), x=normalize(cross([0,0,1],z)), y=cross(z,x);
    return new Float32Array([
      x[0],y[0],z[0],0, x[1],y[1],z[1],0, x[2],y[2],z[2],0,
      -x.reduce((s,v,i)=>s+v*eye[i],0),-y.reduce((s,v,i)=>s+v*eye[i],0),-z.reduce((s,v,i)=>s+v*eye[i],0),1,
    ]);
  };
  const perspective = (aspect) => {
    const near=.01, far=100, f=1/Math.tan(Math.PI/8);
    return new Float32Array([f/aspect,0,0,0, 0,f,0,0, 0,0,(far+near)/(near-far),-1, 0,0,2*far*near/(near-far),0]);
  };
  const multiply = (a,b) => {
    const out=new Float32Array(16);
    for(let c=0;c<4;c++)for(let r=0;r<4;r++)for(let k=0;k<4;k++)out[c*4+r]+=a[k*4+r]*b[c*4+k];
    return out;
  };
  const identity = () => new Float32Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]);
  const poseMatrix = (frame, offset) => new Float32Array([
    frame[offset+3],frame[offset+6],frame[offset+9],0,
    frame[offset+4],frame[offset+7],frame[offset+10],0,
    frame[offset+5],frame[offset+8],frame[offset+11],0,
    frame[offset],frame[offset+1],frame[offset+2],1,
  ]);
  const transform = (m,p) => [m[0]*p[0]+m[4]*p[1]+m[8]*p[2]+m[12],m[1]*p[0]+m[5]*p[1]+m[9]*p[2]+m[13],m[2]*p[0]+m[6]*p[1]+m[10]*p[2]+m[14]];

  class StructuralScene {
    constructor(elements) {
      this.el=elements; this.frameIndex=-1; this.selection='whole'; this.femView='surface'; this.scale='log';
    }

    async load(info) {
      this.info=info;
      const names=['geometry.bin','frames.bin','fem-surface.bin','fem-cutaway.bin'];
      const data=await Promise.all(names.map((name)=>fetchBuffer(info.asset_urls[name])));
      this.geometry=new Float32Array(data[0]); this.frames=new Float32Array(data[1]);
      this.fem={surface:new Float32Array(data[2]),cutaway:new Float32Array(data[3])};
      this.parts=info.parts; this.partIndex=new Map(this.parts.map((part,index)=>[part.id,index]));
      this.sectionParts=this.parts.filter((part)=>part.kind==='section');
      this.bounds=new Map(this.sectionParts.map((part)=>[part.id,this.localBounds(this.geometry,6,part.vertex_offset,part.vertex_count)]));
      this.femBounds={surface:this.localBounds(this.fem.surface,42,0,info.fem_surface_vertex_count),cutaway:this.localBounds(this.fem.cutaway,42,0,info.fem_cutaway_vertex_count)};
      this.setupControls(); this.setupWebGL(); this.seekFrame(0);
    }

    setupControls() {
      this.el.part.replaceChildren();
      const whole=document.createElement('option'); whole.value='whole'; whole.textContent='전체 로봇'; this.el.part.append(whole);
      for(const part of this.parts){const option=document.createElement('option');option.value=part.id;option.textContent=part.label;this.el.part.append(option);}
      this.el.part.addEventListener('change',()=>{this.selection=this.el.part.value;this.updateControls();this.render();});
      this.el.scale.addEventListener('change',()=>{this.scale=this.el.scale.value;this.render();});
      this.el.views.forEach((button)=>button.addEventListener('click',()=>{this.femView=button.dataset.sceneView;this.updateControls();this.render();}));
      this.el.assumptions.replaceChildren(...this.info.assumptions.map((text)=>{const item=document.createElement('li');item.textContent=text;return item;}));
      this.el.scope.textContent=this.info.scope; this.el.root.hidden=false; this.updateControls();
    }

    updateControls() {
      const part=this.parts.find((item)=>item.id===this.selection), fem=part?.kind==='fem';
      this.el.viewGroup.hidden=!fem;
      this.el.views.forEach((button)=>button.setAttribute('aria-pressed',String(button.dataset.sceneView===this.femView)));
      const visualOnly=part?.analysis_status==='visual_only';
      this.el.title.textContent=this.selection==='whole'?'전체 로봇 · 등가 단면 응력 추정':visualOnly?`${part.label} · 비구조 외장 · 응력 미계산`:fem?`${part.label} · 준정적 CAD FEM · 가정`:`${part.label} · 등가 단면 응력 추정`;
      const assumptions=part?.assumptions||this.info.assumptions;
      this.el.partAssumptions.replaceChildren(...assumptions.map((text)=>{const item=document.createElement('li');item.textContent=text;return item;}));
    }

    setupWebGL() {
      const gl=this.el.canvas.getContext('webgl',{antialias:true,alpha:false});
      if(!gl)throw new Error('이 브라우저에서 WebGL을 사용할 수 없습니다.'); this.gl=gl;
      this.program=program(gl,`
        attribute vec3 aPosition; attribute vec3 aNormal; attribute float aStress;
        uniform mat4 uModel; uniform mat4 uViewProjection; uniform float uMaximum; uniform float uLogScale; uniform float uFemMode;
        uniform vec3 uForce; uniform vec3 uMoment; uniform vec3 uOrigin; uniform mat3 uBasis; uniform vec3 uSection;
        varying vec3 vNormal; varying float vValue;
        void main(){gl_Position=uViewProjection*uModel*vec4(aPosition,1.0);vNormal=mat3(uModel)*aNormal;
          float stress;
          if(uFemMode>.5){stress=aStress;}else{
            vec3 q=uBasis*(aPosition-uOrigin);float mx=uMoment.x+q.z*uForce.y;float my=uMoment.y-q.z*uForce.x;
            float sigma=uForce.z/uSection.x+mx*q.y/uSection.y-my*q.x/uSection.y;
            float tx=uForce.x/uSection.x-uMoment.z*q.y/uSection.z;float ty=uForce.y/uSection.x+uMoment.z*q.x/uSection.z;
            stress=sqrt(sigma*sigma+3.0*(tx*tx+ty*ty));
          }
          float linear=clamp(stress/uMaximum,0.0,1.0);float logarithmic=log(1.0+stress/1000.0)/log(1.0+uMaximum/1000.0);
          vValue=mix(linear,logarithmic,uLogScale);}
      `,`
        precision mediump float; varying vec3 vNormal; varying float vValue; uniform float uVisualOnly;
        vec3 ramp(float t){t=clamp(t,0.0,1.0);if(t<.25)return mix(vec3(.08,.25,.58),vec3(.05,.78,.88),t*4.0);
          if(t<.5)return mix(vec3(.05,.78,.88),vec3(.30,.86,.38),(t-.25)*4.0);
          if(t<.75)return mix(vec3(.30,.86,.38),vec3(1.0,.77,.12),(t-.5)*4.0);
          return mix(vec3(1.0,.77,.12),vec3(.93,.12,.12),(t-.75)*4.0);}
        void main(){float light=.42+.58*abs(dot(normalize(vNormal),normalize(vec3(.4,-.5,.75))));vec3 color=uVisualOnly>.5?vec3(.38,.43,.46):ramp(vValue);gl_FragColor=vec4(color*light,1.0);}
      `);
      gl.useProgram(this.program);
      this.locations={
        position:gl.getAttribLocation(this.program,'aPosition'),normal:gl.getAttribLocation(this.program,'aNormal'),stress:gl.getAttribLocation(this.program,'aStress'),
        model:gl.getUniformLocation(this.program,'uModel'),view:gl.getUniformLocation(this.program,'uViewProjection'),maximum:gl.getUniformLocation(this.program,'uMaximum'),log:gl.getUniformLocation(this.program,'uLogScale'),fem:gl.getUniformLocation(this.program,'uFemMode'),visualOnly:gl.getUniformLocation(this.program,'uVisualOnly'),force:gl.getUniformLocation(this.program,'uForce'),moment:gl.getUniformLocation(this.program,'uMoment'),origin:gl.getUniformLocation(this.program,'uOrigin'),basis:gl.getUniformLocation(this.program,'uBasis'),section:gl.getUniformLocation(this.program,'uSection'),
      };
      this.geometryBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,this.geometryBuffer);gl.bufferData(gl.ARRAY_BUFFER,this.geometry,gl.STATIC_DRAW);
      this.stressBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,this.stressBuffer);gl.bufferData(gl.ARRAY_BUFFER,this.geometry.length/6*4,gl.DYNAMIC_DRAW);
      this.femBuffers={}; for(const name of ['surface','cutaway']){const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,this.fem[name],gl.STATIC_DRAW);this.femBuffers[name]=buffer;}
      gl.enable(gl.DEPTH_TEST);gl.disable(gl.CULL_FACE);gl.clearColor(.012,.022,.029,1);
      if(typeof ResizeObserver==='function'){
        this.resizeObserver=new ResizeObserver(()=>this.render());this.resizeObserver.observe(this.el.canvas);
      }else{
        window.addEventListener('resize',()=>this.render());
      }
    }

    seekFrame(index, videoTime) {
      if(!this.info)return;
      const bounded=Math.min(this.info.frame_count-1,Math.max(0,Number(index)|0));
      if(Number.isFinite(Number(videoTime)))this.el.canvas.dataset.syncVideoTime=Number(videoTime).toFixed(6);
      if(bounded===this.frameIndex)return;
      this.frameIndex=bounded;this.el.canvas.dataset.frameIndex=String(bounded);this.render();
    }

    femStress(part, data) {
      const partOffset=this.frameIndex*this.info.frame_stride_floats+this.partIndex.get(part.id)*19,c0=this.frames[partOffset+12],c1=this.frames[partOffset+13],c2=this.frames[partOffset+14],c3=this.frames[partOffset+15],c4=this.frames[partOffset+16],c5=this.frames[partOffset+17];
      const count=data.length/42,stress=new Float32Array(count);
      for(let vertex=0;vertex<count;vertex++){
        const b=vertex*42+6,xx=c0*data[b]+c1*data[b+6]+c2*data[b+12]+c3*data[b+18]+c4*data[b+24]+c5*data[b+30],yy=c0*data[b+1]+c1*data[b+7]+c2*data[b+13]+c3*data[b+19]+c4*data[b+25]+c5*data[b+31],zz=c0*data[b+2]+c1*data[b+8]+c2*data[b+14]+c3*data[b+20]+c4*data[b+26]+c5*data[b+32],xy=c0*data[b+3]+c1*data[b+9]+c2*data[b+15]+c3*data[b+21]+c4*data[b+27]+c5*data[b+33],yz=c0*data[b+4]+c1*data[b+10]+c2*data[b+16]+c3*data[b+22]+c4*data[b+28]+c5*data[b+34],zx=c0*data[b+5]+c1*data[b+11]+c2*data[b+17]+c3*data[b+23]+c4*data[b+29]+c5*data[b+35],value=Math.sqrt(.5*((xx-yy)**2+(yy-zz)**2+(zz-xx)**2)+3*(xy*xy+yz*yz+zx*zx));
        stress[vertex]=value;
      } return stress;
    }

    localBounds(data,stride,offset,count){const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];for(let i=0;i<count;i++){const base=(offset+i)*stride;for(let a=0;a<3;a++){min[a]=Math.min(min[a],data[base+a]);max[a]=Math.max(max[a],data[base+a]);}}return {min,max};}
    includeBounds(total,bounds,matrix){for(const x of [bounds.min[0],bounds.max[0]])for(const y of [bounds.min[1],bounds.max[1]])for(const z of [bounds.min[2],bounds.max[2]]){const p=transform(matrix,[x,y,z]);for(let a=0;a<3;a++){total.min[a]=Math.min(total.min[a],p[a]);total.max[a]=Math.max(total.max[a],p[a]);}}}
    viewProjection(bounds){const center=bounds.min.map((v,i)=>(v+bounds.max[i])/2),radius=Math.max(.08,Math.hypot(...bounds.max.map((v,i)=>v-bounds.min[i]))/2),eye=[center[0]+radius*1.5,center[1]-radius*2.1,center[2]+radius*1.15];return multiply(perspective(this.el.canvas.width/this.el.canvas.height),lookAt(eye,center));}

    bindGeometry(buffer,stride,withStress) {
      const gl=this.gl;gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.enableVertexAttribArray(this.locations.position);gl.vertexAttribPointer(this.locations.position,3,gl.FLOAT,false,stride*4,0);gl.enableVertexAttribArray(this.locations.normal);gl.vertexAttribPointer(this.locations.normal,3,gl.FLOAT,false,stride*4,12);
      if(withStress){gl.bindBuffer(gl.ARRAY_BUFFER,this.stressBuffer);gl.enableVertexAttribArray(this.locations.stress);gl.vertexAttribPointer(this.locations.stress,1,gl.FLOAT,false,4,0);}else{gl.disableVertexAttribArray(this.locations.stress);gl.vertexAttrib1f(this.locations.stress,0);}
    }

    render() {
      if(!this.gl||this.frameIndex<0)return;const renderStarted=performance.now(),gl=this.gl,canvas=this.el.canvas,ratio=Math.min(window.devicePixelRatio||1,2),width=Math.max(2,Math.round(canvas.clientWidth*ratio)),height=Math.max(2,Math.round(canvas.clientHeight*ratio));
      if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height;}gl.viewport(0,0,width,height);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.useProgram(this.program);
      gl.uniform1f(this.locations.maximum,this.info.max_von_mises_pa);gl.uniform1f(this.locations.log,this.scale==='log'?1:0);
      const selected=this.parts.find((part)=>part.id===this.selection),frameBase=this.frameIndex*this.info.frame_stride_floats,totalBounds={min:[Infinity,Infinity,Infinity],max:[-Infinity,-Infinity,-Infinity]};let maximum=0;
      if(selected?.kind==='fem'){
        const data=this.fem[this.femView],stress=this.femStress(selected,data),offset=this.partIndex.get(selected.id)*19;maximum=this.frames[frameBase+offset+18];gl.bindBuffer(gl.ARRAY_BUFFER,this.stressBuffer);gl.bufferData(gl.ARRAY_BUFFER,stress,gl.DYNAMIC_DRAW);this.bindGeometry(this.femBuffers[this.femView],42,true);gl.uniform1f(this.locations.fem,1);gl.uniform1f(this.locations.visualOnly,0);gl.uniformMatrix4fv(this.locations.model,false,identity());this.includeBounds(totalBounds,this.femBounds[this.femView],identity());gl.uniformMatrix4fv(this.locations.view,false,this.viewProjection(totalBounds));gl.drawArrays(gl.TRIANGLES,0,data.length/42);
      }else{
        const shown=selected?[selected]:this.sectionParts;
        for(const part of shown){const offset=this.partIndex.get(part.id)*19;if(part.analysis_status!=='visual_only')maximum=Math.max(maximum,this.frames[frameBase+offset+18]);const model=selected?identity():poseMatrix(this.frames,frameBase+offset);this.includeBounds(totalBounds,this.bounds.get(part.id),model);}
        this.bindGeometry(this.geometryBuffer,6,false);gl.uniform1f(this.locations.fem,0);gl.uniformMatrix4fv(this.locations.view,false,this.viewProjection(totalBounds));
        for(const part of shown){const offset=this.partIndex.get(part.id)*19,s=part.section;gl.uniform1f(this.locations.visualOnly,part.analysis_status==='visual_only'?1:0);gl.uniform3fv(this.locations.force,this.frames.subarray(frameBase+offset+12,frameBase+offset+15));gl.uniform3fv(this.locations.moment,this.frames.subarray(frameBase+offset+15,frameBase+offset+18));gl.uniform3fv(this.locations.origin,s.origin_body_m);gl.uniformMatrix3fv(this.locations.basis,false,s.basis_body);gl.uniform3f(this.locations.section,s.area_m2,s.I_m4,s.J_m4);gl.uniformMatrix4fv(this.locations.model,false,selected?identity():poseMatrix(this.frames,frameBase+offset));gl.drawArrays(gl.TRIANGLES,part.vertex_offset,part.vertex_count);}
      }
      const time=this.info.frame_times_s[this.frameIndex],visualOnly=selected?.analysis_status==='visual_only';this.el.clock.textContent=`기록 프레임 ${this.frameIndex+1} / ${this.info.frame_count} · ${time.toFixed(3)} s`;
      this.el.maximum.textContent=visualOnly?'현재 최대 · 미계산':`${selected?.kind==='fem'?'현재 전체 FEM 최대':'현재 최대'} ${maximum/1e6<.01?(maximum/1e3).toFixed(2)+' kPa':(maximum/1e6).toFixed(3)+' MPa'}`;
      this.el.legend.textContent=visualOnly?'회색 · 비구조 외장 · 응력 미계산':`0 — ${(this.info.max_von_mises_pa/1e6).toFixed(2)} MPa · ${this.scale==='log'?'로그':'선형'} 색상`;
      if(visualOnly){this.el.force.textContent='비구조 외장 · 미적용';this.el.moment.textContent='비구조 외장 · 미적용';}else if(selected){const offset=frameBase+this.partIndex.get(selected.id)*19,f=this.frames.subarray(offset+12,offset+15),m=this.frames.subarray(offset+15,offset+18);this.el.force.textContent=`Fx ${f[0].toFixed(2)} · Fy ${f[1].toFixed(2)} · Fz ${f[2].toFixed(2)} N`;this.el.moment.textContent=`Mx ${m[0].toFixed(3)} · My ${m[1].toFixed(3)} · Mz ${m[2].toFixed(3)} N·m`;}else{this.el.force.textContent='부품 선택 시 표시';this.el.moment.textContent='부품 선택 시 표시';}
      const elapsed=performance.now()-renderStarted,previous=Number(canvas.dataset.maxRenderMs)||0;canvas.dataset.lastRenderMs=elapsed.toFixed(3);canvas.dataset.maxRenderMs=Math.max(previous,elapsed).toFixed(3);
    }
  }

  window.StructuralScene=StructuralScene;
})();
