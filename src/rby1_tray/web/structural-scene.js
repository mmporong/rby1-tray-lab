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
  const fmt = (value,digits=2) => Number(value).toLocaleString('ko-KR',{maximumFractionDigits:digits,minimumFractionDigits:digits});
  const stressText = (value) => Math.abs(value)<1e4?`${fmt(value/1e3)} kPa`:`${fmt(value/1e6,3)} MPa`;
  const componentOptions = {total:'합성 등가응력',axial:'축력만 · |σN|',bending:'굽힘만 · |σB|',shear:'횡전단만 · √3|τV|',torsion:'비틀림만 · √3|τT|',normaldev:'법선 편차 성분',femshear:'좌표축 전단 성분'};
  const bodyLabel = (text) => ({left_ft_mount:'왼손목 FT 연결부',right_ft_mount:'오른손목 FT 연결부'}[text]||text);
  const element = (tag,text,className) => {const node=document.createElement(tag);if(text!==undefined)node.textContent=text;if(className)node.className=className;return node;};

  class StructuralScene {
    constructor(elements) {
      this.el=elements; this.frameIndex=-1; this.selection='whole'; this.femView='surface'; this.scale='log';this.component='total';this.jointStateFrame=-1;
    }

    async load(info) {
      this.info=info;
      const names=['geometry.bin','frames.bin','fem-surface.bin','fem-cutaway.bin'];
      const data=await Promise.all(names.map((name)=>fetchBuffer(info.asset_urls[name])));
      this.geometry=new Float32Array(data[0]); this.frames=new Float32Array(data[1]);
      this.fem={surface:new Float32Array(data[2]),cutaway:new Float32Array(data[3])};
      if(info.load_diagnostics){this.loads=new Float32Array(await fetchBuffer(info.load_diagnostics.asset_url));}
      this.parts=info.parts; this.partIndex=new Map(this.parts.map((part,index)=>[part.id,index]));
      this.sectionParts=this.parts.filter((part)=>part.kind==='section');
      this.bounds=new Map(this.sectionParts.map((part)=>[part.id,this.localBounds(this.geometry,6,part.vertex_offset,part.vertex_count)]));
      this.femBounds={surface:this.localBounds(this.fem.surface,42,0,info.fem_surface_vertex_count),cutaway:this.localBounds(this.fem.cutaway,42,0,info.fem_cutaway_vertex_count)};
      this.setupDiagnostics();this.setupJointSummary();this.setupControls(); this.setupWebGL(); this.seekFrame(0);
    }

    setupJointSummary() {
      if(!this.el.jointSummary)return;
      this.jointStatus=element('p',undefined,'joint-load-status');this.jointTop=element('div',undefined,'joint-load-top');
      const details=element('details'),summary=element('summary','몸통·양팔 20개 관절 수치'),table=element('table');
      table.innerHTML='<thead><tr><th scope="col">관절</th><th scope="col">토크 Nm</th><th scope="col">모델 제한 Nm</th><th scope="col">사용률 %</th></tr></thead>';
      this.jointRows=element('tbody');table.append(this.jointRows);const wrap=element('div',undefined,'joint-load-table');wrap.append(table);details.append(summary,wrap);
      this.jointDetails=details;this.el.jointSummary.replaceChildren(this.jointStatus,this.jointTop,details);
      this.el.jointToggle?.addEventListener('change',()=>this.render());
    }

    setJointState(state,index) {
      this.jointState=state;this.jointStateFrame=Number(index);this.jointDirty=true;
    }

    setupDiagnostics() {
      if(!this.el.diagnostic)return;
      const controls=element('div',undefined,'structural-mechanism-controls'),label=element('label','응력 성분 보기');
      this.componentSelect=element('select');this.componentSelect.id='structuralComponent';this.componentSelect.setAttribute('aria-label','응력 성분 보기');label.append(this.componentSelect);controls.append(label);
      this.componentSelect.addEventListener('change',()=>{this.component=this.componentSelect.value;this.render();});
      this.diagnosticBody=element('div');
      this.diagnosticBody.innerHTML=`<p class="structural-diagnostic-summary" data-focus></p><div class="structural-hot-parts" aria-label="현재 응력이 큰 부품"></div>
        <dl class="structural-load-grid">
          <div class="structural-load-metric"><dt>축력 N · z방향</dt><dd data-axial></dd><small>길이축 방향 힘</small></div>
          <div class="structural-load-metric"><dt>횡전단력 V · xy</dt><dd data-shear></dd><small>길이축에 수직인 힘</small></div>
          <div class="structural-load-metric"><dt>굽힘 모멘트 Mb · xy</dt><dd data-bending></dd><small>길이축을 휘게 하는 모멘트</small></div>
          <div class="structural-load-metric"><dt>비틀림 T · z축</dt><dd data-torsion></dd><small>길이축 주위 모멘트</small></div>
        </dl>
        <p class="structural-axis-readout" data-origin></p>
        <details class="structural-explain"><summary>어디서 전달된 하중인가</summary><p data-path></p><p data-load-note></p>
          <div class="structural-wrench-table"><table><caption>같은 원점·좌표계의 하중 분해</caption><thead><tr><th scope="col">지지 하중 항</th><th scope="col">Fx N</th><th scope="col">Fy N</th><th scope="col">Fz N</th><th scope="col">Mx N·m</th><th scope="col">My N·m</th><th scope="col">Mz N·m</th></tr></thead><tbody data-sources></tbody></table></div>
        </details>
        <details class="structural-explain"><summary>표시점의 응력과 계산 기준</summary><p data-point></p><p data-components></p><p data-properties></p>
          <p>성분 보기는 해당 항만 남긴 가상 응력입니다. 부호에 따른 상쇄가 있어 단독 등가응력을 더하거나 기여율로 읽을 수 없습니다. 파손·안전율 판정은 포함하지 않습니다.</p>
        </details>`;
      this.el.diagnostic.replaceChildren(controls,this.diagnosticBody);
      this.fields=Object.fromEntries(['focus','axial','shear','bending','torsion','origin','path','load-note','sources','point','components','properties'].map((key)=>[key,this.diagnosticBody.querySelector(`[data-${key}]`)]));
      this.hotParts=this.diagnosticBody.querySelector('.structural-hot-parts');
      this.timelinePeaks=new Map(this.info.parts.map((part,i)=>{let value=-1,index=0;for(let n=0;n<this.info.frame_count;n++){const peak=this.frames[n*this.info.frame_stride_floats+i*19+18];if(peak>value){value=peak;index=n;}}return [part.id,{value,index}];}));
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
      if(this.componentSelect){const allowed=fem?['total','normaldev','femshear']:['total','axial','bending','shear','torsion'];if(!allowed.includes(this.component))this.component='total';this.componentSelect.replaceChildren(...allowed.map((key)=>{const option=element('option',componentOptions[key]);option.value=key;return option;}));this.componentSelect.value=this.component;this.componentSelect.disabled=visualOnly;}
    }

    setupWebGL() {
      const gl=this.el.canvas.getContext('webgl',{antialias:true,alpha:true});
      if(!gl)throw new Error('이 브라우저에서 WebGL을 사용할 수 없습니다.'); this.gl=gl;
      this.program=program(gl,`
        attribute vec3 aPosition; attribute vec3 aNormal; attribute float aStress;
        uniform mat4 uModel; uniform mat4 uViewProjection; uniform float uMaximum; uniform float uLogScale; uniform float uFemMode;
        uniform vec3 uForce; uniform vec3 uMoment; uniform vec3 uOrigin; uniform mat3 uBasis; uniform vec3 uSection;uniform float uComponent;
        varying vec3 vNormal; varying float vValue;
        void main(){gl_Position=uViewProjection*uModel*vec4(aPosition,1.0);vNormal=mat3(uModel)*aNormal;
          float stress;
          if(uFemMode>.5){stress=aStress;}else{
            vec3 q=uBasis*(aPosition-uOrigin);float mx=uMoment.x+q.z*uForce.y;float my=uMoment.y-q.z*uForce.x;
            float axial=uForce.z/uSection.x;float bending=mx*q.y/uSection.y-my*q.x/uSection.y;
            vec2 shear=uForce.xy/uSection.x;vec2 torsion=vec2(-uMoment.z*q.y,uMoment.z*q.x)/uSection.z;
            stress=sqrt((axial+bending)*(axial+bending)+3.0*dot(shear+torsion,shear+torsion));
            if(uComponent>.5&&uComponent<1.5)stress=abs(axial);
            else if(uComponent>1.5&&uComponent<2.5)stress=abs(bending);
            else if(uComponent>2.5&&uComponent<3.5)stress=sqrt(3.0)*length(shear);
            else if(uComponent>3.5)stress=sqrt(3.0)*length(torsion);
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
        model:gl.getUniformLocation(this.program,'uModel'),view:gl.getUniformLocation(this.program,'uViewProjection'),maximum:gl.getUniformLocation(this.program,'uMaximum'),log:gl.getUniformLocation(this.program,'uLogScale'),fem:gl.getUniformLocation(this.program,'uFemMode'),visualOnly:gl.getUniformLocation(this.program,'uVisualOnly'),force:gl.getUniformLocation(this.program,'uForce'),moment:gl.getUniformLocation(this.program,'uMoment'),origin:gl.getUniformLocation(this.program,'uOrigin'),basis:gl.getUniformLocation(this.program,'uBasis'),section:gl.getUniformLocation(this.program,'uSection'),component:gl.getUniformLocation(this.program,'uComponent'),
      };
      this.geometryBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,this.geometryBuffer);gl.bufferData(gl.ARRAY_BUFFER,this.geometry,gl.STATIC_DRAW);
      this.stressBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,this.stressBuffer);gl.bufferData(gl.ARRAY_BUFFER,this.geometry.length/6*4,gl.DYNAMIC_DRAW);
      this.femBuffers={}; for(const name of ['surface','cutaway']){const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,this.fem[name],gl.STATIC_DRAW);this.femBuffers[name]=buffer;}
      this.floorProgram=program(gl,`
        attribute vec2 aPosition;uniform mat4 uViewProjection;uniform mediump vec2 uCenter;uniform mediump float uRadius;varying mediump vec2 vWorld;
        void main(){vWorld=aPosition*uRadius+uCenter;gl_Position=uViewProjection*vec4(vWorld,-.001,1.);}
      `,`
        precision mediump float;varying vec2 vWorld;uniform vec2 uCenter;uniform vec2 uBase;uniform float uRadius;
        void main(){vec2 cell=abs(fract(vWorld/.25-.5)-.5)*.25;float line=1.-smoothstep(.001,.004,min(cell.x,cell.y));
          float checker=mod(floor(vWorld.x/.25)+floor(vWorld.y/.25),2.);vec3 color=mix(vec3(.35,.43,.48),vec3(.40,.49,.54),checker);
          color=mix(color,vec3(.57,.69,.73),line*.38);float shadow=exp(-dot(vWorld-uBase,vWorld-uBase)/.13);color*=1.-shadow*.16;
          float alpha=1.-smoothstep(uRadius*.55,uRadius,length(vWorld-uCenter));gl_FragColor=vec4(color,alpha);}
      `);
      this.floorLocations={position:gl.getAttribLocation(this.floorProgram,'aPosition'),view:gl.getUniformLocation(this.floorProgram,'uViewProjection'),center:gl.getUniformLocation(this.floorProgram,'uCenter'),base:gl.getUniformLocation(this.floorProgram,'uBase'),radius:gl.getUniformLocation(this.floorProgram,'uRadius')};
      this.floorBuffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,this.floorBuffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array([-1,-1,1,-1,1,1,-1,-1,1,1,-1,1]),gl.STATIC_DRAW);
      gl.enable(gl.DEPTH_TEST);gl.disable(gl.CULL_FACE);gl.clearColor(0,0,0,0);
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
      if(bounded===this.frameIndex&&!this.jointDirty)return;
      this.jointDirty=false;
      this.frameIndex=bounded;this.el.canvas.dataset.frameIndex=String(bounded);this.render();
    }

    drawEnvironment(projection,bounds) {
      const gl=this.gl,l=this.floorLocations,center=[(bounds.min[0]+bounds.max[0])/2,(bounds.min[1]+bounds.max[1])/2],baseIndex=this.partIndex.get('base'),baseOffset=this.frameIndex*this.info.frame_stride_floats+(baseIndex||0)*19;
      gl.useProgram(this.floorProgram);gl.uniformMatrix4fv(l.view,false,projection);gl.uniform2fv(l.center,center);gl.uniform2f(l.base,this.frames[baseOffset],this.frames[baseOffset+1]);gl.uniform1f(l.radius,4);
      gl.bindBuffer(gl.ARRAY_BUFFER,this.floorBuffer);gl.enableVertexAttribArray(l.position);gl.vertexAttribPointer(l.position,2,gl.FLOAT,false,8,0);gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE_MINUS_SRC_ALPHA);gl.drawArrays(gl.TRIANGLES,0,6);gl.disable(gl.BLEND);gl.useProgram(this.program);
    }

    femStress(part, data) {
      const partOffset=this.frameIndex*this.info.frame_stride_floats+this.partIndex.get(part.id)*19,c0=this.frames[partOffset+12],c1=this.frames[partOffset+13],c2=this.frames[partOffset+14],c3=this.frames[partOffset+15],c4=this.frames[partOffset+16],c5=this.frames[partOffset+17];
      const count=data.length/42,stress=new Float32Array(count);let sampleMaximum=-1;
      for(let vertex=0;vertex<count;vertex++){
        const b=vertex*42+6,xx=c0*data[b]+c1*data[b+6]+c2*data[b+12]+c3*data[b+18]+c4*data[b+24]+c5*data[b+30],yy=c0*data[b+1]+c1*data[b+7]+c2*data[b+13]+c3*data[b+19]+c4*data[b+25]+c5*data[b+31],zz=c0*data[b+2]+c1*data[b+8]+c2*data[b+14]+c3*data[b+20]+c4*data[b+26]+c5*data[b+32],xy=c0*data[b+3]+c1*data[b+9]+c2*data[b+15]+c3*data[b+21]+c4*data[b+27]+c5*data[b+33],yz=c0*data[b+4]+c1*data[b+10]+c2*data[b+16]+c3*data[b+22]+c4*data[b+28]+c5*data[b+34],zx=c0*data[b+5]+c1*data[b+11]+c2*data[b+17]+c3*data[b+23]+c4*data[b+29]+c5*data[b+35],value=Math.sqrt(.5*((xx-yy)**2+(yy-zz)**2+(zz-xx)**2)+3*(xy*xy+yz*yz+zx*zx));
        const normal=Math.sqrt(.5*((xx-yy)**2+(yy-zz)**2+(zz-xx)**2)),shear=Math.sqrt(3*(xy*xy+yz*yz+zx*zx));
        stress[vertex]=this.component==='normaldev'?normal:this.component==='femshear'?shear:value;
        if(value>sampleMaximum){sampleMaximum=value;this.femSample={point:Array.from(data.subarray(vertex*42,vertex*42+3)),tensor:[xx,yy,zz,xy,yz,zx]};}
      } return stress;
    }

    sectionPeak(part,force,moment) {
      const key=`${part.id}:${this.frameIndex}`;if(this.sectionSample?.key===key)return this.sectionSample;
      const s=part.section,b=s.basis_body,o=s.origin_body_m,g=this.geometry;let maximum=-1,point=[0,0,0],q=[0,0,0];
      const axial=force[2]/s.area_m2,sx=force[0]/s.area_m2,sy=force[1]/s.area_m2;
      for(let n=part.vertex_offset;n<part.vertex_offset+part.vertex_count;n++){
        const dx=g[n*6]-o[0],dy=g[n*6+1]-o[1],dz=g[n*6+2]-o[2],x=b[0]*dx+b[3]*dy+b[6]*dz,y=b[1]*dx+b[4]*dy+b[7]*dz,z=b[2]*dx+b[5]*dy+b[8]*dz;
        const sigma=axial+((moment[0]+z*force[1])*y-(moment[1]-z*force[0])*x)/s.I_m4,tx=sx-moment[2]*y/s.J_m4,ty=sy+moment[2]*x/s.J_m4,value=sigma*sigma+3*(tx*tx+ty*ty);
        if(value>maximum){maximum=value;point=[g[n*6],g[n*6+1],g[n*6+2]];q=[x,y,z];}
      }
      this.sectionSample={key,point,q};return this.sectionSample;
    }

    updateDiagnostics(selected,frameBase,projection) {
      if(!this.fields)return;
      const ranked=this.sectionParts.filter(p=>p.analysis_status!=='visual_only').map(p=>({part:p,value:this.frames[frameBase+this.partIndex.get(p.id)*19+18]})).sort((a,b)=>b.value-a.value),part=selected||ranked[0]?.part;
      if(!part)return;
      const offset=frameBase+this.partIndex.get(part.id)*19,force=this.frames.subarray(offset+12,offset+15),moment=this.frames.subarray(offset+15,offset+18),visualOnly=part.analysis_status==='visual_only',fem=part.kind==='fem',d=window.StructuralDiagnostics;
      this.fields.focus.textContent=`${selected?'선택 부품':'현재 합성응력이 가장 큰 부품'} · ${part.label}`;
      if(this.hotKey!==ranked.slice(0,3).map(x=>x.part.id).join(',')){
        this.hotKey=ranked.slice(0,3).map(x=>x.part.id).join(',');
        this.hotParts.replaceChildren(...ranked.slice(0,3).map(({part:p},i)=>{const button=element('button',`${i+1}. ${p.label}`);button.type='button';button.addEventListener('click',()=>{this.selection=p.id;this.el.part.value=p.id;this.updateControls();this.render();});return button;}));
      }
      if(visualOnly){for(const key of ['axial','shear','bending','torsion'])this.fields[key].textContent='미계산';this.fields.path.textContent='내력 경로가 정의되지 않은 무질량 외장입니다.';this.fields.origin.textContent='응력 0 또는 안전 상태를 뜻하지 않습니다.';for(const key of ['load-note','point','components','properties'])this.fields[key].textContent='';this.fields.sources.replaceChildren();this.el.overlay?.replaceChildren();return;}
      const r=d.resultant(force,moment);this.fields.axial.textContent=`${fmt(r.axial_n)} N`;this.fields.shear.textContent=`${fmt(r.shear_n)} N`;this.fields.bending.textContent=`${fmt(r.bending_nm,3)} N·m`;this.fields.torsion.textContent=`${fmt(r.torsion_nm,3)} N·m`;
      const meta=this.info.load_diagnostics?.parts[this.partIndex.get(part.id)],source=meta?.source_body_per_frame?.[this.frameIndex],mass=source?meta?.subtree_mass_kg_by_source?.[source]:meta?.subtree_mass_kg;
      const worldOrigin=fem?Array.from(this.frames.subarray(offset,offset+3)):transform(poseMatrix(this.frames,offset),part.section.origin_body_m);
      this.fields.origin.textContent=`합력 기준 원점 · 세계 [${worldOrigin.map(x=>fmt(x,3)).join(', ')}] m · ${fem?'CAD 축':'단면 축'} z=길이축, x·y=횡방향. +/−는 표시축 방향입니다.`;
      this.fields.path.textContent=meta?`${bodyLabel(meta.parent_label)} ↔ ${bodyLabel(source?(meta.source_labels?.[source]||source):meta.source_body_label)} · ${meta.action==='child_on_parent'?'하부가 상부에 가하는 반작용':'부모가 하부 구조를 지지하는 합력'}${Number.isFinite(mass)?` · 포함된 로봇 하부 질량 ${fmt(mass,3)} kg`:''}`:'부모–자식 연결부를 통과하는 6축 합력입니다. 원인별 분해 데이터는 이 번들에 없습니다.';
      this.fields['load-note'].textContent=this.loads?'자중 지지 + 외력 반작용 + 운동 잔차 = 현재 합력. 외력 항은 접촉·적용 외력·공간 구속을 포함합니다. 운동 잔차는 앞 두 항을 뺀 값이며 독립적인 원인별 기여율이 아닙니다. 적재물 하중은 접촉을 통해 전달되며 위 로봇 질량에는 포함되지 않습니다.':'자중·운동·접촉이 반영된 합력이며, 이 값만으로 각각의 원인을 분리하지 않습니다.';
      const rows=[];if(this.loads){const start=this.frameIndex*this.info.load_diagnostics.frame_stride_floats+this.partIndex.get(part.id)*18;for(let i=0;i<3;i++)rows.push([['자중 지지','외력 반작용','운동 잔차'][i],Array.from(this.loads.subarray(start+i*6,start+i*6+6))]);}
      rows.push(['현재 합력',[...force,...moment]]);this.fields.sources.replaceChildren(...rows.map(([label,values])=>{const tr=element('tr');tr.append(element('th',label));values.forEach((v,i)=>tr.append(element('td',fmt(v,i<3?2:3))));return tr;}));
      const peak=this.timelinePeaks.get(part.id);let point;
      if(fem&&this.femSample){const metrics=d.femTensorMetrics(this.femSample.tensor);point=this.femSample.point;this.fields.point.textContent=`표시 ${this.femView==='cutaway'?'단면':'표면'}의 합성 VM 최대 표본 · CAD [${point.map(x=>fmt(x*1000,2)).join(', ')}] mm · ${stressText(metrics.von_mises_pa)}. 전체 요소 최대와는 별도입니다.`;this.fields.components.textContent=`같은 점의 법선 편차 ${stressText(metrics.normal_deviatoric_vm_pa)} · 좌표축 전단 ${stressText(metrics.shear_vm_pa)} · 평균 법선응력 ${stressText(metrics.hydrostatic_pa)}`;this.fields.properties.textContent='FEM 좌표축 전단은 τxy·τyz·τzx의 조합입니다. 단면의 횡전단력 V와 같은 항이 아닙니다. E=69 GPa, ν=0.33·상단 고정·좌표 정합 가정.';}
      else if(!fem&&selected){const sample=this.sectionPeak(part,force,moment),metrics=d.sectionPoint(sample.q,force,moment,part.section);point=sample.point;this.fields.point.textContent=`표시 외피의 합성 VM 최대점 · 단면 [${sample.q.map(x=>fmt(x*1000,1)).join(', ')}] mm · ${stressText(metrics.total_pa)}`;this.fields.components.textContent=`같은 점의 부호 있는 법선응력: 축력 ${stressText(metrics.axial_pa)} + 굽힘 ${stressText(metrics.bending_pa)}. 단독 등가응력: 횡전단 ${stressText(metrics.shear_vm_pa)} · 비틀림 ${stressText(metrics.torsion_vm_pa)}.`;this.fields.properties.textContent=`가정 단면 A=${fmt(part.section.area_m2*1e6)} mm² · I=${fmt(part.section.I_m4*1e12)} mm⁴ · J=${fmt(part.section.J_m4*1e12)} mm⁴. 횡전단 V/A는 공칭 평균이며 분포 체적력·국부 체결 응력은 복원하지 않습니다.`;}
      else{this.fields.point.textContent='목록이나 위 순위의 부품을 선택하면 같은 프레임의 상세 분해와 응력 최대점을 표시합니다.';this.fields.components.textContent='축력·굽힘은 법선응력을, 횡전단·비틀림은 전단응력을 만듭니다. 성분 선택으로 각 항의 분포를 비교할 수 있습니다.';this.fields.properties.textContent='전신은 가정 단면 추정이며 전체 CAD FEM이 아닙니다.';}
      if(peak)this.fields.properties.textContent+=` · 전 구간 합성 VM 최대 ${stressText(peak.value)} / ${fmt(this.info.frame_times_s[peak.index],3)} s`;
      this.drawPeakPoint(projection,point);
    }

    drawPeakPoint(projection,point) {
      const svg=this.el.overlay;if(!svg)return;svg.replaceChildren();if(!point)return;
      const w=projection[3]*point[0]+projection[7]*point[1]+projection[11]*point[2]+projection[15];if(w<=0)return;
      const x=(projection[0]*point[0]+projection[4]*point[1]+projection[8]*point[2]+projection[12])/w*500+500,y=500-(projection[1]*point[0]+projection[5]*point[1]+projection[9]*point[2]+projection[13])/w*500;
      svg.setAttribute('viewBox','0 0 1000 1000');
      const line=document.createElementNS('http://www.w3.org/2000/svg','line');
      for(const [key,value]of Object.entries({x1:x,y1:y,x2:x+35,y2:y-35,stroke:'#dde6f2','stroke-width':2}))line.setAttribute(key,String(value));
      const text=document.createElementNS('http://www.w3.org/2000/svg','text');
      for(const [key,value]of Object.entries({x:x+39,y:y-35,fill:'#dde6f2','font-size':21}))text.setAttribute(key,String(value));
      text.textContent='최대점';svg.append(line,text);
    }

    drawJointOverlay(projection,selected) {
      const svg=this.el.jointOverlay;if(!svg)return;svg.replaceChildren();
      const current=this.jointStateFrame===this.frameIndex,loads=window.StructuralDiagnostics.jointLoads(current?this.jointState:null,this.info.joint_load),enabled=this.el.jointToggle?.checked!==false;
      svg.hidden=!enabled;svg.dataset.frameIndex=current?String(this.frameIndex):'';svg.dataset.jointCount='0';svg.dataset.jointCalloutCount='0';
      this.jointDetails.hidden=!loads.available;
      if(!loads.available){this.jointStatus.textContent=`관절 부하 표시 불가 · ${current?loads.reason:'같은 기록 프레임의 관절 상태 대기'}`;this.jointTop.replaceChildren();this.jointTopKey=null;if(this.el.jointLegend)this.el.jointLegend.textContent='관절 데이터 없음 · 0%로 대체하지 않습니다.';this.el.canvas.dataset.jointFrameIndex='';return;}
      this.el.canvas.dataset.jointFrameIndex=String(this.jointStateFrame);
      const ranked=loads.joints.slice().sort((a,b)=>b.load_fraction-a.load_fraction),peak=ranked[0];
      this.jointStatus.textContent=`최대 · ${peak.label} · ${fmt(peak.load_fraction*100,1)}% · τ ${fmt(peak.torque_nm)} Nm / 제한 ${fmt(peak.limit_nm,0)} Nm`;
      const keys=ranked.slice(0,3).map(j=>j.name).join(',');if(keys!==this.jointTopKey){this.jointTopKey=keys;this.jointTop.replaceChildren(...ranked.slice(0,3).map(j=>{const button=element('button');button.type='button';button.dataset.joint=j.name;button.addEventListener('click',()=>{if(!this.partIndex.has(j.part_id))return;this.selection=j.part_id;this.el.part.value=j.part_id;this.updateControls();this.render();});return button;}));}
      [...this.jointTop.children].forEach((button,i)=>{const j=ranked[i];button.textContent=`${j.label} · ${fmt(j.load_fraction*100,1)}%`;});
      if(!this.jointCells){this.jointCells=new Map();this.jointRows.replaceChildren(...loads.joints.map(j=>{const row=element('tr'),cells=[element('th',j.label),element('td'),element('td'),element('td')];row.append(...cells);this.jointCells.set(j.name,cells);return row;}));}
      for(const j of loads.joints){const cells=this.jointCells.get(j.name);cells[1].textContent=fmt(j.torque_nm);cells[2].textContent=fmt(j.limit_nm,0);cells[3].textContent=fmt(j.load_fraction*100,1);}
      const fem=selected?.kind==='fem',shown=fem?[]:loads.joints.filter(j=>this.partIndex.has(j.part_id)&&(!selected||j.part_id===selected.id));
      if(this.el.jointLegend)this.el.jointLegend.textContent=`관절 부하 · 20개 관절 기록${fem?' · CAD 상세에서는 지시선 생략':enabled?'':' · 지시선 꺼짐'}`;
      if(!enabled||fem)return;
      const ns='http://www.w3.org/2000/svg',width=svg.clientWidth||this.el.canvas.clientWidth,height=svg.clientHeight||this.el.canvas.clientHeight,make=(tag,attrs,text)=>{const node=document.createElementNS(ns,tag);for(const [key,value]of Object.entries(attrs))node.setAttribute(key,String(value));if(text)node.textContent=text;svg.append(node);return node;};
      svg.setAttribute('viewBox',`0 0 ${width} ${height}`);const points=new Map();
      for(const j of shown){const offset=this.frameIndex*this.info.frame_stride_floats+this.partIndex.get(j.part_id)*19,world=transform(selected?identity():poseMatrix(this.frames,offset),j.anchor_body_m),w=projection[3]*world[0]+projection[7]*world[1]+projection[11]*world[2]+projection[15];if(w<=0)continue;
        const x=((projection[0]*world[0]+projection[4]*world[1]+projection[8]*world[2]+projection[12])/w*.5+.5)*width,y=(.5-(projection[1]*world[0]+projection[5]*world[1]+projection[9]*world[2]+projection[13])/w*.5)*height;
        if(x<0||x>width||y<0||y>height)continue;points.set(j.name,[x,y]);

      }
      svg.dataset.jointCount=String(points.size);
      const labelJoints=(selected?shown:ranked).filter(j=>points.has(j.name)).slice(0,width<500?1:3);
      svg.dataset.jointCalloutCount=String(labelJoints.length);
      if(this.el.jointLegend)this.el.jointLegend.textContent=`관절 부하 · 전체 20개 중 ${selected?'선택 관절':'상위 부하'} ${labelJoints.length}개 지시선`;
      labelJoints.forEach((j,i)=>{const p=points.get(j.name),x=width-154,y=16+i*48;make('polyline',{'data-joint':j.name,points:`${p[0]},${p[1]} ${x-10},${y+17} ${x},${y+17}`,fill:'none',stroke:j.load_fraction>=1?'#ff557b':'#d8b6fc','stroke-width':1.25,opacity:.9});make('rect',{x,y,width:144,height:38,rx:3,fill:'#26394b',opacity:.94});make('text',{x:x+8,y:y+14,fill:'#e6d3ff','font-size':11},j.label);make('text',{x:x+8,y:y+30,fill:'#fff','font-size':12,'font-weight':700},`${fmt(j.load_fraction*100,1)}% · ${fmt(j.torque_nm,1)} Nm`);});
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
      gl.uniform1f(this.locations.component,Math.max(0,['total','axial','bending','shear','torsion'].indexOf(this.component)));
      const selected=this.parts.find((part)=>part.id===this.selection),frameBase=this.frameIndex*this.info.frame_stride_floats,totalBounds={min:[Infinity,Infinity,Infinity],max:[-Infinity,-Infinity,-Infinity]};let maximum=0,projection;
      if(selected?.kind==='fem'){
        const data=this.fem[this.femView],stress=this.femStress(selected,data),offset=this.partIndex.get(selected.id)*19;maximum=this.frames[frameBase+offset+18];gl.bindBuffer(gl.ARRAY_BUFFER,this.stressBuffer);gl.bufferData(gl.ARRAY_BUFFER,stress,gl.DYNAMIC_DRAW);this.bindGeometry(this.femBuffers[this.femView],42,true);gl.uniform1f(this.locations.fem,1);gl.uniform1f(this.locations.visualOnly,0);gl.uniformMatrix4fv(this.locations.model,false,identity());this.includeBounds(totalBounds,this.femBounds[this.femView],identity());projection=this.viewProjection(totalBounds);gl.uniformMatrix4fv(this.locations.view,false,projection);gl.drawArrays(gl.TRIANGLES,0,data.length/42);
      }else{
        const shown=selected?[selected]:this.sectionParts;
        for(const part of shown){const offset=this.partIndex.get(part.id)*19;if(part.analysis_status!=='visual_only')maximum=Math.max(maximum,this.frames[frameBase+offset+18]);const model=selected?identity():poseMatrix(this.frames,frameBase+offset);this.includeBounds(totalBounds,this.bounds.get(part.id),model);}
        projection=this.viewProjection(totalBounds);if(!selected)this.drawEnvironment(projection,totalBounds);this.bindGeometry(this.geometryBuffer,6,false);gl.uniform1f(this.locations.fem,0);gl.uniformMatrix4fv(this.locations.view,false,projection);
        for(const part of shown){const offset=this.partIndex.get(part.id)*19,s=part.section;gl.uniform1f(this.locations.visualOnly,part.analysis_status==='visual_only'?1:0);gl.uniform3fv(this.locations.force,this.frames.subarray(frameBase+offset+12,frameBase+offset+15));gl.uniform3fv(this.locations.moment,this.frames.subarray(frameBase+offset+15,frameBase+offset+18));gl.uniform3fv(this.locations.origin,s.origin_body_m);gl.uniformMatrix3fv(this.locations.basis,false,s.basis_body);gl.uniform3f(this.locations.section,s.area_m2,s.I_m4,s.J_m4);gl.uniformMatrix4fv(this.locations.model,false,selected?identity():poseMatrix(this.frames,frameBase+offset));gl.drawArrays(gl.TRIANGLES,part.vertex_offset,part.vertex_count);}
      }
      const time=this.info.frame_times_s[this.frameIndex],visualOnly=selected?.analysis_status==='visual_only';this.el.clock.textContent=`기록 프레임 ${this.frameIndex+1} / ${this.info.frame_count} · ${time.toFixed(3)} s`;
      this.el.maximum.textContent=visualOnly?'현재 최대 · 미계산':`${selected?.kind==='fem'?'현재 전체 FEM 최대':'현재 합성 VM 최대'} ${maximum/1e6<.01?(maximum/1e3).toFixed(2)+' kPa':(maximum/1e6).toFixed(3)+' MPa'}`;
      this.el.legend.textContent=visualOnly?'회색 · 비구조 외장 · 응력 미계산':`${componentOptions[this.component]} · 0 — ${(this.info.max_von_mises_pa/1e6).toFixed(2)} MPa · ${this.scale==='log'?'로그':'선형'} 색상`;
      if(visualOnly){this.el.force.textContent='비구조 외장 · 미적용';this.el.moment.textContent='비구조 외장 · 미적용';}else if(selected){const offset=frameBase+this.partIndex.get(selected.id)*19,f=this.frames.subarray(offset+12,offset+15),m=this.frames.subarray(offset+15,offset+18);this.el.force.textContent=`Fx ${f[0].toFixed(2)} · Fy ${f[1].toFixed(2)} · Fz ${f[2].toFixed(2)} N`;this.el.moment.textContent=`Mx ${m[0].toFixed(3)} · My ${m[1].toFixed(3)} · Mz ${m[2].toFixed(3)} N·m`;}else{this.el.force.textContent='부품 선택 시 표시';this.el.moment.textContent='부품 선택 시 표시';}
      this.updateDiagnostics(selected,frameBase,projection);this.drawJointOverlay(projection,selected);canvas.dataset.component=this.component;
      const elapsed=performance.now()-renderStarted,previous=Number(canvas.dataset.maxRenderMs)||0;canvas.dataset.lastRenderMs=elapsed.toFixed(3);canvas.dataset.maxRenderMs=Math.max(previous,elapsed).toFixed(3);
    }
  }

  window.StructuralScene=StructuralScene;
})();
