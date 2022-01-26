# ICCPS_REP_SUB221

Summary

Resilient cyber-physical systems (CPS) must ensure safety and perform required tasks in the presence of malicious cyber attacks. Recently, restart-based defenses have been proposed in which a CPS mitigates attacks by reverting to an initial safe state. In our
paper [1], we consider a class of reactive restart approaches for CPS under malicious attacks with verifiable safety guarantees. We consider a setting where the controllers are engineered to crash and reboot following faults or attacks. We present a hybraid system model that captures the trade-off between security, availability, and safety of the CPS due to the reactive restart. We develop sufficient conditions under which an affine controller provides verifiable safety guarantees for the physical plant using a barrier certificate approach. We synthesize safety-critical controllers using control barrier functions to guarantee system safety under given timing parameters. We present two case studies on the proposed approach using a warehouse temperature control system and a two-dimensional non-linear system. Our proposed approach in [1] guarantees the safety for both cases.

Code description

The provided scripts include the MATLAB (version 2019a) implementation of the Algorithm 1 for two case studies (Warehouse Temperature Control System and System with Polynomial Dynamics) and the related results presented in the Simulation Case Studies (Section 7) of [1]. Detailed description of the MATLAB scripts are given below.

• GitHub link: https://github.com/sdinuka/ICCPS_REP_SUB221

• MATLAB version - 2019a

• MATLAB toolboxes required - symbolic toolbox; optimization toolbox; control system toolbox

• MATLAB third party toolboxes required to download - YALMIP toolbox [2] (install via tbxmanager [3]); SeDuMi [4]; SOSTOOLS [5] (version .303)

• Steps for using the provided MATLAB scripts for generating the results (Figure 4 and Figure 5) presented in [1]:

 (***) Please follow the steps below to generate the results presented in Figure 4 of [1]
 
 (-) First run WTC_Sys_Algorithm1.m to generate corresponding timing parameters eta, tau, and phi.

 (-) Then to generate the results presented in Figure 4a of [1] run "WTC_Sys_Affine_Control.m"
 
 (-) To generate the results presented in Figure 4b of [1] run "WTC_Sys_SC_Control.m"
 
 (***) Please follow the steps below to generate the results presented in Figure 5 of [1]
 
 (-) First run Poly_Sys_Algorithm1.m to generate corresponding timing parameters eta, tau, and phi.
 
 (-)  Then to generate the results presented in Figure 5a of [1] run "Poly_Sys_Affine_Control.m"
 
 (-) To generate the results presented in Figure 5b of [1] run "Poly_Sys_SC_Control.m"
 
 • Detailed Summary of the scripts:

 (-) WTC_Sys_Algorithm1.m --> Computes the timing parameters eta (initialization window), tau (exploit window), and phi (vulnerability window) using Algorithm 1 in [1] for the Case Study 1 presented in section 7.1 of [1]
 
 (-) WTC_Sys_Affine_Control.m --> Generates the results Figure 4a in section 7.1 of [1]
 
 (-) WTC_Sys_SC_Control.m --> Generates the results Figure 4b in section 7.1 of [1]
 
 (-) Poly_Sys_Find_CBF.m --> Computes the control barrier function for the Case Study 2 presented in section 7.2 of [1]
 
 (-) Poly_Sys_Algorithm1.m --> Computes the timing parameters eta (initialization window), tau (exploit window), and phi (vulnerability window) using Algorithm 1 in [1] for the Case Study 2 presented in section 7.2 of [1]
 
 (-) Poly_Sys_Affine_Control.m --> Generates the results Figure 5a in section 7.2 of [1]
 
 (-) Poly_Sys_SC_Control.m --> Generates the results Figure 5b in section 7.2 of [1]

***** Virtual Machine details *****

(***) A Virtual Machine installed with all the dependencies required for running the provided MATLAB scripts is accessible through [6]

 (-) Virtual machine generated using VirtualBox (version 6.1)
 
 (-) Settings of the Host machine used for running the provided virtual machine: a workstation with Intel(R) Xeon(R) W-2145 CPU with 3.70GHz processor and 128GB memory running MS Windows 10.
 
 (-) To import the virtual machine, first download and unzip the folder "ICCPS_REP_SUB221_VM.rar" at [6] and then import the file "ICCPS_REP_SUB221.ova" (File type: OVF-Open Virtualization Format Package) using VirtualBox
 
 (-) Open MATLAB installed in the virtual machine and navigate to the "ICCPC_REP_SUB221" folder to locate all the MATLAB scripts that are needed to generate the results given in [1].

Note: You may freely redistribute and use this sample code, with or without modification, provided you include the original Copyright notice and use restrictions.

Disclaimer: THE SAMPLE CODE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DINUKA SAHABANDU OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) SUSTAINED BY YOU OR A THIRD PARTY, HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT ARISING IN ANY WAY OUT OF THE USE OF THIS SAMPLE CODE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Related papers

[1] L.Niu, D.Sahabandu, A.Clark, and R.Poovendran, "Verifying Safety for Resilient Cyber-Physical Systems via Reactive Software Restart." In Proceedings of 13th ACM/IEEE International Conference on Cyber-Physical Systems (ICCPS), Milan, Italy, May 2022.

[2] https://yalmip.github.io/tutorial/installation/

[3] http://tbxmanager.com

[4] https://github.com/sqlp/sedumi

[5] https://www.cds.caltech.edu/sostools/

[6] https://www.dropbox.com/sh/o8wg283sslcxy9z/AACwSMDFsDcD55jGlYSk-AY0a?dl=0

For additional information, contact: Dinuka Sahabandu, email: sdinuka@uw.edu

Acknowledgement: This work was supported by AFOSR grant FA9550-20-1-0074.
