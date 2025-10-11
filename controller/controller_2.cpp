#include <controller_2.h>
#include "matplotlibcpp.h"
#include "KalmanFilter.h"
#include <kinematics_model.h>
#include <string>
#include <fstream>

namespace plt = matplotlibcpp;

controller_2::controller_2(/* args */)
{   

    // M_d_a << 0.5, 0., 0., 0.5;
    // D_d_a << 1.0, 0., 0., 1.0;
    // K_d_a << 2.0, 0., 0., 2.0;
    // M << 0.1, 0., 0., 0.1;
    // K << 5000., 0., 0., 5000.;
    // B << 30., 0., 0., 30.;
    // L << 0., 0., 0., 0.;

    // M_d_a << 1.0, 0., 0., 1.0;
    // D_d_a << 2.61, 0., 0., 2.61;
    // K_d_a << 5., 0., 0., 5.;
    // M << 0.5, 0., 0., 0.5;
    // M << 3., 0., 0., 3.;

    // M << 0.5, 0., 0., 0.5;
    // K << 800., 0., 0., 800.;
    // B << 60., 0., 0., 60.;
    // L << 5., 0., 0., 5.;

    M_d_a << 1., 0., 0., 1.;
    // D_d_a << 1.5, 0., 0., 1.5;
    D_d_a << 1.5, 0., 0., 1.5;
    K_d_a << 4.0, 0., 0., 4.0;
    // M_d_a << 1., 0., 0., 1.;
    // D_d_a << 1.5, 0., 0., 1.5;
    // K_d_a << 2.0, 0., 0., 2.0;

    // M << 2.5, 0., 0., 2.5;
    M << 0.5, 0., 0., 0.5;
    K << 800., 0., 0., 800.;
    B << 60., 0., 0., 60.;
    L << 5., 0., 0., 5.;


    // Lamda_1 << 40., 0., 0., 40.;
    // Lamda_2 << 40., 0., 0., 40.;
    // F0 << 1.0, 1.0;
    // F_max << 6., 6.;
    // M_d_a << 1.0, 0., 0., 1.0;
    // D_d_a << 2.0, 0., 0., 2.0;
    // K_d_a << 5.0, 0., 0., 5.0;

    M_s << 2.5, 0., 0., 2.5;
    // M_s << 0.5, 0., 0., 0.5;
    // D_s << 10., 0., 0., 10.;
    // D_s << 1., 0., 0., 1.; // good
    // D_s << 0.2, 0., 0., 0.2; // good
    // D_s << 0.15, 0., 0., 0.15;
    D_s << 5., 0., 0., 5.;
    // D_s << 6., 0., 0., 6.;
    M_s_hat << 0.5, 0., 0., 0.5;
    D_s_hat << 10., 0., 0., 10.;

    Lamda_1 << 20., 0., 0., 20.;
    Lamda_2 << 40., 0., 0., 40.;
    // Lamda_1 << 10., 0., 0., 10.;
    // Lamda_2 << 10., 0., 0., 10.;
    F0 << 4.52, 4.52;
    // F0 << 4., 4.;
    F_max << 5., 5.;
    // F0 << 6.1, 6.1;
    // F_max << 8., 8.;

    // M_s << 0., 0., 0., 0.;
    // D_s << 0., 0., 0., 0.;
    // Lamda_2 << 20., 0., 0., 20.;
    K_star << 10., 0., 0., 10.;

    q0.push_back(init_angle);
    q0_dot.push_back(Eigen::Vector2d::Zero());
    q0_ddot.push_back(Eigen::Vector2d::Zero());
    X.push_back(init_pos);
    X_d.push_back(init_pos);

    X0.push_back(init_pos); 
    X0_dot.push_back(Eigen::Vector2d::Zero());
    X0_ddot.push_back(Eigen::Vector2d::Zero());

    u_x_star.push_back(Eigen::Vector2d::Zero());
    q_x_star.push_back(Eigen::Vector2d::Zero());
    phi_b.push_back(Eigen::Vector2d::Zero());
    phi_a.push_back(Eigen::Vector2d::Zero());
    q_star.push_back(Eigen::Vector2d::Zero());

    tau_star.push_back(Eigen::Vector2d::Zero());
    tau.push_back(Eigen::Vector2d::Zero());
    q_x.push_back(init_angle);
    q_x_dot.push_back(Eigen::Vector2d::Zero());
    u_x.push_back(Eigen::Vector2d::Zero());
    a.push_back(Eigen::Vector2d::Zero());


    e_r.push_back(Eigen::Vector2d::Zero());
    e_q.push_back(Eigen::Vector2d::Zero());
    v_x.push_back(Eigen::Vector2d::Zero());

    q.push_back(init_angle);
    f_ext.push_back(Eigen::Vector2d::Zero());
    tau_ext.push_back(Eigen::Vector2d::Zero());

    frame = 0;


    e_v_ML.push_back(Eigen::Vector2d::Zero());
    f_e_ML.push_back(Eigen::Vector2d::Zero());

    A_dot_ML.setZero();
    A_ML.setZero();

    Tau_ML << 10.0, 0.0, 0.0, 10.0;
    
    omiga1_ML << 0.023521286, -0.03498055, -0.069849156, 0.13349184, 
                -0.020947043, 0.023344383, 0.011615698, -0.026089743, 
                0.19448292, -0.1566046, -0.059964117, 0.021470515, 
                0.20809998, 0.03980558, 0.05612056, -0.0027422616, 
                -0.118113816, 0.0033436934, 0.021548383, -0.15709484, 
                0.04732817, -0.20253465, 0.0557766, -0.037039544, 
                -0.2284308, -0.14104953, -0.036825217, 0.09759844, 
                -0.24009082, 0.12088176, -0.12535898, -0.14892583, 
                0.10978829, -0.1657396, -0.07561633, -0.08281792, 
                -0.116494715, 0.1640805, -0.06216812, -0.13376103, 
                0.023352098, 0.04739395, -0.051483873, -0.031830017, 
                -0.05606274, 0.01490505, -0.03822417, 0.07079971, 
                0.0032872907, 0.10141573, 0.1105021, -0.1256955, 
                -0.16244993, 0.06804768, -0.087957606, -0.06525439, 
                -0.09497231, 0.013661729, 0.039277375, 0.07334009, 
                0.009741961, -0.15322137, 0.17360401, 0.10104212, 
                0.27281916, -0.03205112, 0.015824309, 0.031848054, 
                -0.055420686, -0.10972247, -0.07326113, 0.20278238, 
                0.038149633, -0.043169297, 0.105906725, -0.093066044, 
                0.16290161, 0.003206886, 0.045511566, 0.11894938, 
                0.06877007, -0.130888, -0.02473798, -0.0073241354, 
                -0.37108114, -0.040286556, 0.06863002, -0.08794501, 
                -0.046425845, 0.0061464454, -0.06465768, 0.052881893, 
                0.01938217, -0.04026644, -0.14564064, -0.06131211, 
                -0.17725703, -0.052291527, 0.013478395, 0.09387221, 
                0.12140125, -0.09838838, -0.12817526, 0.04471195, 
                0.11577616, 0.028988402, -0.1405045, 0.051449664, 
                0.03663319, -0.030774608, -0.11524376, 0.0015259064, 
                0.08639639, -0.05002006, -0.04805073, 0.041041296, 
                -0.035304558, 0.29495418, 0.03750452, -0.08539129, 
                0.0355571, -0.11339727, 0.05859734, -0.054873455, 
                0.05265026, -0.080230355, -0.0067370716, 0.14242955;

     omiga2_ML << 0.058569334, -0.08420738, -0.12361756, 0.015644003, -0.18687013, 0.017062925, -0.026215535, 0.16237897, -0.015693743, 0.07005593, 0.050578967, 0.032478787, 0.23966031, 0.15033388, 0.007032559, 0.15372893, 0.022377726, 0.09388125, 0.13275936, -0.0685532, 0.14321539, -0.07631945, -0.04823347, 0.13426228, -0.18601342, 0.059479587, -0.15969996, 0.020347988, 0.030913198, 0.085914254, -0.17162885, -0.110178, 
0.098562755, 0.07560295, 0.08548203, -0.08404517, -0.21362732, 0.3249114, 0.08321195, -0.014182987, -0.015256625, -0.08247639, 0.01890448, 0.19531903, 0.03519486, -0.10526665, 0.18396117, 0.14011195, 0.35345957, 0.11820198, -0.085953586, 0.08679705, -0.06390112, 0.010742653, 0.012251576, 0.08565837, 0.07619038, -0.027491104, -0.082651176, -0.024791863, 0.1480224, 0.09755568, 0.11795277, 0.04963354, 
-0.09559556, -0.013925161, -0.23911992, 0.11573484, 0.043066032, -0.11543836, -0.0014380123, 0.09998295, 0.2175579, -0.10092048, 0.14534543, -0.037233595, -0.009632417, 0.094703436, 0.20040782, -0.03446071, -0.23563875, 0.0029707393, -0.120022, -0.1280451, -0.030806992, 0.0949941, 0.079558924, 0.022896016, 0.03735224, -0.17871694, -0.02358877, 0.11532313, 0.0019679633, -0.05091917, -0.13100912, -0.009787172, 
-0.06220031, 0.0189096, 0.13381037, -0.09793898, -0.086714916, -0.23193924, 0.21707587, -0.18747863, 0.070301004, -0.04039273, 0.26131785, -0.0063910694, 0.015840398, -0.16641289, -0.048074216, -0.08501951, 0.072120026, -0.067631036, -0.0045321626, -0.066625245, -0.10809387, -0.014788448, -0.017888268, -0.019796137, 0.021185748, 0.016123632, 0.020667795, 0.07950874, 0.20859802, -0.099461555, 0.006504067, 0.21099968, 
-0.14340776, -0.0028143753, 0.10537814, -0.010497707, -0.021706821, -0.07336106, -0.06607305, -0.029944256, 0.004547744, -0.016036514, 0.04790554, 0.039962333, 0.12608656, -0.15782009, 0.07019714, 0.23504435, -0.06395685, 0.20599924, 0.16728327, 0.104458086, -0.019884977, -0.08601729, -0.024010997, 0.022173632, -0.15024583, -0.23295578, 0.08673855, 0.15413879, 0.10825621, 0.010560209, 0.18339801, 0.14624597, 
-0.08159451, 0.0138000995, -0.043096144, 0.03467102, -0.052330352, -0.2086395, -0.14468205, 0.15628093, -0.008757237, -0.07320376, 0.14867894, 0.024357576, 0.14745729, -0.107831515, 0.06840028, -0.05337413, 0.11615545, -0.15162106, 0.040051147, 0.13985473, -0.05369673, 0.12359183, -0.33109945, 0.0012940015, 0.017786413, -0.018612605, 0.053324517, -0.07687158, -0.01609257, 0.08178839, 0.18022223, -0.08364176, 
-0.31567344, -0.043340407, -0.122945644, -0.052205544, 0.019158982, -0.05123716, 0.08116762, 0.19463134, -0.18751082, -0.025329603, 0.21206544, 0.095407724, -0.043763984, -0.04145201, -0.050012905, 0.054468986, -0.083422355, -0.13524853, -0.04005484, -0.18048257, 0.034652308, 0.040630117, -0.089146495, -0.17855586, -0.0038736681, -0.03523074, -0.097505845, -0.12434299, -0.039944306, -0.00678243, -0.026549188, -0.025276309, 
0.09233549, -0.20024939, 0.080526076, 0.06826763, -0.038348377, -0.06550295, 0.025939258, -0.18639535, 0.2505986, -0.22280638, -0.09500923, -0.029162921, 0.08979645, -0.016885519, -0.0445143, 0.08082665, 0.18618265, 0.09291423, -0.15571907, 0.03282073, 0.1500755, 0.19473264, -0.10339981, 0.0018726755, -0.17808016, 0.15459764, -0.041598223, 0.08843732, -0.16852504, -0.085076034, -0.03271618, 0.07913798, 
0.045374293, -0.011001228, -0.05388192, -0.012482018, -0.048731077, 0.020138748, -0.04544494, 0.121395476, -0.049786046, -0.05709788, 0.07264927, 0.052510586, 0.24926758, 0.13857979, -0.12937559, 0.18421625, -0.036560316, -0.10800497, -0.17333908, -0.26756936, -0.14124192, 0.10833923, 0.035096545, -0.010948291, 0.12384685, -0.10526581, 0.021386702, 0.097434156, 0.026736462, -0.058280077, -0.22370382, -0.0044945395, 
0.011647943, 0.07607808, 0.063202664, -0.1606301, -0.15384732, 0.13604939, -0.017829625, 0.075905666, 0.09312738, -0.019455507, -0.013975448, -0.014631761, -0.010683149, 0.018929824, 0.0020050146, 0.12464754, -0.051975578, 0.010104413, 0.041381262, 0.14180984, 0.011482941, 0.040447805, -0.15850493, -0.13680133, 0.18672384, 0.10059029, -0.026231565, -0.122032516, -0.08690396, 0.06194755, 0.11986893, 0.021528699, 
-0.17896205, 0.08239981, -0.06369403, -0.053142134, -0.007996005, -0.15475772, 0.08052303, 0.09348032, -0.14336088, -0.05338744, 0.04570552, -0.1188686, 0.16907756, 0.09593046, 0.066502295, -0.14770949, -0.15865442, 0.0091527905, 0.06819549, -0.15100475, -0.10276607, 0.2144282, -0.058575556, 0.1498887, 0.11523636, -0.040204544, -0.21934949, -0.01770103, -0.069919765, -0.07176214, 0.0123782605, -0.08431954, 
-0.24457602, -0.14734544, -0.12557516, -0.20870833, -0.0645572, -0.09608762, 0.017878337, 0.07808507, -0.06121122, 0.013682123, 0.042238034, -0.13081914, -0.23621258, 0.00044071535, -0.04213594, -0.06709973, -0.19308682, -0.11144078, 0.15111737, -0.010672562, -0.1640479, 0.17708607, -0.026261155, -0.021970686, -0.05737193, 0.000381812, -0.14784439, -0.071605, -0.053822868, 0.034209356, 0.1306555, -0.048857674, 
-0.06576911, 0.18191108, -0.124417685, 0.100947015, 0.14065163, -0.124738194, 0.05926084, -0.038366728, -0.061917942, 0.115944885, 0.104609855, 0.037677858, 0.20084324, -0.02826691, -0.027008526, -0.04990509, -0.10123493, 0.08816927, -0.022706524, -0.12013638, 0.056271534, 0.02247597, 0.04901545, 0.08835065, 0.094816245, 0.06773451, 0.010450924, 0.02076399, -0.0737621, -0.05044335, 0.08176811, 0.03131403, 
-0.25794113, 0.11943536, 0.055294413, -0.026764434, 0.14155756, 0.030393766, 0.14490892, 0.10565452, -0.021818802, 0.06664427, 0.0028847456, 0.018420143, 0.046120852, 0.03901776, 0.056047432, -0.04175178, -0.09860917, -0.23066194, -0.103686176, -0.068171114, -0.08049605, -0.04430313, 0.067137025, 0.0410443, 0.05887174, -0.19872148, 0.032482952, 0.13448852, -0.005001996, 0.0057225097, -0.16783682, -0.34684545, 
-0.010630459, 0.054689396, 0.17754622, -0.11602292, 0.099322945, 0.16615416, -0.08592218, -0.106756575, 0.14273952, 0.12750033, -0.040845633, -0.003172244, -0.0041793548, 0.14637892, -0.09720762, 0.081748635, 0.14921153, 0.15455899, -0.044912387, 0.049995456, 0.080653384, -0.012021307, 0.09678867, 0.11615458, 0.25927967, -0.09524789, 0.004558116, -0.04121988, 0.021952491, 0.023869287, 0.030738378, 0.098817214, 
-0.17975578, 0.09369958, 0.07550452, 0.12386461, -0.03174472, -0.1313748, 0.09630022, 0.04820821, -0.16013967, -0.07069385, 0.08587885, -0.032459103, 0.23198417, 0.07690658, 0.13205901, -0.17111923, -0.16052078, 0.02357969, -0.02649593, -0.1758238, -0.063234866, 0.15434332, -0.05962497, -0.062704034, 0.14323212, -0.06394814, -0.057851046, 0.10032732, 0.020684939, 0.006496056, -0.17069402, -0.03510712, 
0.028195865, -0.005472606, -0.0807184, -0.12576875, -0.030753328, -0.026165584, 0.2312019, 0.11078626, 0.1787383, 0.12125753, 0.09729763, 0.03546803, 0.15799363, -0.13007891, -0.06683904, -0.04558983, -0.13825981, -0.03166545, 0.12152245, -0.09695478, -0.15736593, 0.007691992, -0.09477483, 0.123428084, -0.026243236, -0.053246856, 0.12542507, -0.15987942, -0.19964656, 0.08324321, 0.05926692, -0.18375696, 
0.1258364, -0.020277468, 0.13677073, 0.0517923, -0.058669537, 0.012792178, -0.093996346, -0.048965562, 0.14502273, -0.016986884, 0.010552362, -0.118866086, -0.012547432, -0.07079366, -0.07104863, 0.09193688, 0.12451192, 0.12823755, 0.16764635, 0.08968756, 0.12882233, -0.14676559, 0.2758988, 0.16376486, -0.07128173, -0.10373964, 0.0062565254, 0.10544052, 0.102985494, -0.26024702, -0.11434171, 0.11888569, 
0.11895741, 0.021402998, 0.13834935, 0.11154945, 0.09916864, -0.041981068, 0.096834965, 0.017996587, -0.028493196, 0.03942512, -0.029221345, -0.03080628, -0.15247147, -0.099856675, 0.10658715, 0.040583387, 0.08630796, 0.18350373, 0.06331045, 0.028378878, 0.055117153, 0.08257044, -0.06808568, 0.14122742, 0.058820665, 0.15989372, -0.050278008, 0.14196026, 0.012555567, -0.06925973, -0.02589064, 0.11118493, 
0.049647402, -0.05970408, -0.037648953, -0.009351078, -0.10337218, -0.03793159, -0.04314348, -0.21087609, -0.004452738, -0.09225314, -0.017933298, 0.061294995, -0.020520907, -0.1392353, -0.10850665, 0.052916873, 0.045666866, 0.10481985, -0.025406033, -0.031228947, -0.07665123, -0.06598401, 0.19197607, -0.09958918, 0.09082444, 0.10949829, 0.05410396, -0.08490529, -0.14054589, -0.13093229, -0.0015128474, 0.07267772, 
-0.06893445, 0.27372056, 0.0033096378, -0.1156189, -0.075899445, 0.01880842, -0.119730845, -0.017933805, 0.12504032, 0.039621625, -0.14769268, -0.032664597, 0.087027706, 0.15037206, -0.07039424, -0.11380281, -0.006640042, -0.09863112, -0.08950718, -0.2052157, -0.089971, 0.16434757, -0.12771682, 0.032834273, 0.21609575, -0.023838198, -0.04233401, -0.045587566, -0.07216565, 0.051162235, 0.009000018, -0.02656803, 
0.08818822, -0.024105184, 0.0028861887, 0.03246488, -0.17148119, -0.031815365, -0.05196659, -0.11925199, 0.007924015, 0.1726174, -0.04629026, -0.11598061, -0.057830345, -0.17099823, 0.04599047, 0.09881717, 0.092269205, 0.13553613, 0.16645063, 0.14629562, -0.1781696, -0.022037322, -0.05970648, -0.10156821, -0.16621241, -0.008696056, 0.019105576, -0.1056334, 0.12532629, -0.019730607, 0.10515603, 0.011750979, 
0.12898983, -0.09022585, 0.04540942, 0.015580228, 0.008165398, 0.050803095, -0.1461391, -0.17001629, -0.25091302, -0.0021450822, -0.06478202, -0.095793486, -0.007677741, -0.15814179, -0.054449398, 0.0026381272, -0.015825456, 0.07869079, 0.111052774, 0.1200061, 0.13718978, 0.02314226, -0.19623715, 0.18381745, 0.17781049, 0.06470378, 0.008323209, -0.080572926, 0.16937304, 0.007984203, 0.035621017, 0.02243029, 
-0.07418605, -0.065862305, -0.023321433, -0.016126366, 0.23484956, -0.13166615, -0.095797636, 0.06709122, -0.18808126, 0.10540827, 0.027936067, 0.050640702, -0.03268983, 0.078533575, -0.028522637, -0.12452178, -0.2646906, -0.12530991, -0.16154252, -0.15489365, -0.1514902, 0.026655763, -0.16198428, -0.15621404, -0.11205289, 0.05784814, -0.0009380714, 0.040802877, 0.030661553, 0.01945764, 0.03815189, -0.031430345, 
0.11521354, 0.12203062, 0.015516415, -0.22233614, 0.1635994, -0.043391712, -0.073671356, 0.14943796, -0.1087818, -0.16531983, -0.0879044, 0.21676005, 0.09014274, -0.020045282, -0.035466637, 0.12769106, -0.25833127, 0.011067789, -0.0029033092, 0.098166324, -0.03494392, 0.120482326, 0.012181948, 0.05115028, 0.11303966, 0.13288234, -0.18412244, 0.12811038, -0.048226874, 0.2025395, 0.1079627, -0.025810527, 
0.017164104, -0.25684586, 0.111650236, 0.06435197, -0.16679804, 0.11634995, 0.17077036, 0.003416839, 0.030771287, -0.04185567, -0.013592772, 0.055729833, -0.022388864, 0.037636712, -0.0925403, 0.19788915, -0.0070735468, 0.04558757, 0.10041477, -0.13855192, -0.093379036, -0.22009519, 0.03322728, -0.013113042, 0.13945472, 0.008135969, 0.30334097, 0.06920688, 0.008593695, -0.07474927, -0.11269542, 0.07162044, 
0.0916938, -0.016873287, 0.037514284, -0.017168835, -0.073206395, 0.04970026, -0.070975825, 0.01356268, -0.14675021, -0.035579737, -0.006809494, 0.08087486, -0.15957847, -0.044763237, 0.09231183, 0.13086627, 0.14246668, 0.054229666, 0.083895706, 0.09079314, 0.06408764, -0.008898458, -0.010718549, 0.09142554, -0.015706655, 0.062858485, -0.13762978, 0.020857537, 0.058400936, 0.025494706, 0.12002458, 0.214515, 
-0.03590463, -0.029417135, 0.10411631, -0.0616624, -0.077554494, -0.037197966, 0.0744544, -0.001399871, 0.09579229, -0.062607616, -0.00635646, 0.012910675, -0.04748848, 0.21196468, 0.03611477, -0.09337902, 0.022869818, 0.07819095, 0.0040013697, -0.21122283, 0.03322904, -0.11206974, -0.11656823, -0.03711062, 0.0033449363, -0.14224271, 0.059505336, 0.09612423, 0.033182252, 0.16132577, 0.008108723, 0.042915452, 
-0.10924034, 0.12295557, -0.20305851, -0.05619988, -0.007796861, -0.009157378, 0.17059995, -0.06968309, -0.087063745, 0.14498606, 0.11703559, 0.09482817, 0.105028346, -0.008550688, -0.079138555, -0.13843554, -0.25591984, -0.08490278, -0.031761754, -0.07417547, -0.029315837, 0.0945118, -0.14116232, -0.00064689247, -0.061487228, -0.047416516, -0.020937696, -0.00793123, -0.16741739, 0.10069591, -0.059146635, -0.038955133, 
0.084422916, -0.09503856, -0.00595949, -0.005797208, 0.039813668, -0.094475165, 0.07430486, 0.078330845, -0.08377709, 0.033796933, -0.016081735, 0.10171479, -0.0064399876, 0.11381659, 0.13632326, 0.034102917, 0.0056797913, -0.12570164, 0.06320824, 0.20840047, -0.0041642776, 0.24530014, -0.02442807, -0.06873423, 0.033351753, -0.1678869, -0.1407343, -0.001273558, -0.09720548, -0.079192504, -0.20011324, -0.040644098, 
-0.03207932, 0.016579337, 0.01988427, -0.04208116, -0.0323394, 0.10788506, 0.11794716, -0.022719204, 0.05318328, 0.008234713, -0.12217321, 0.082284406, 0.06277838, -0.13533849, 0.07227448, 0.117186874, -0.29678074, 0.2128872, 0.16711412, 0.031418584, -0.011144638, -0.182284, 0.09508006, -0.037933134, -0.035403404, -0.07366821, -0.101877615, 0.023365289, -0.06550684, -0.0017602872, 0.06043259, 0.09232525, 
0.09785573, 0.0329804, -0.01844332, -0.090168945, -0.0123424465, -0.09358265, 0.16790627, 0.24107578, -0.07855875, 0.092929594, 0.01980885, 0.1027702, -0.039060898, 0.16698673, -0.018005686, -0.045686204, -0.09652763, -0.12630619, 0.080116235, -0.22004738, -0.10778441, 0.008730511, 0.10125778, 0.044454686, 0.33062816, 0.086202875, -0.12636611, 0.2030912, -0.110199116, -0.073152885, 0.103472054, -0.084754586;

b1_ML << -0.072415315, 0.21288826, 0.06860185, 0.048354466, 0.23762503, -0.04863656, 0.25405574, 0.021471629, 0.120672934, -0.07648071, 0.10877328, -0.07770306, -0.018303676, -0.031125791, -0.113577604, -0.2859347, 0.05104528, -0.18259919, -0.14033872, -0.041742943, -0.05777675, 0.09397955, -0.21896067, 0.011965184, 0.028246485, 0.07306072, -0.015882045, 0.066000946, -0.10815375, 0.25616938, -0.0998017, -0.038714174;

b2_ML << 0.1480433, 0.03417773, 0.03384673, -0.16284849, 0.11760352, 0.03767746, -0.027358664, -0.08312049, 0.011446045, -0.1907786, -0.099205814, 0.017207889, -0.023713421, -0.14333698, 0.05122103, -0.052959472, 0.00076523656, 0.26659992, -0.13605578, 0.050713852, -0.047260735, 0.06962526, -0.008225264, -0.034686334, -0.032746404, 0.07137805, 0.004294276, 0.114852555, -0.17576791, 7.985113e-05, -0.025554147, 0.004393501;


    generateJointTrajectory();
    plotGeneratedTrajectory();

}

void controller_2::getfe(vv2d &q, const double &T)
{
     Eigen::Matrix<double, 32,1> y;
    Eigen::Matrix<double, 32,1> y_1;
    Eigen::Vector4d x;
    if(frame == 0)
    {
        x << q[frame][0], q[frame][1], q[frame][0]  / T , q[frame ][1] / T;

    }else{

        x << q[frame][0], q[frame][1], (q[frame][0] - q[frame-1][0]) / T , (q[frame ][1] - q[frame-1][1]) / T;
    }

    y_1 = omiga1_ML*x + b1_ML;
    for(int i = 0; i < 32; i++)
    {
        y_1(i,0) = tanh(y_1(i,0));
    }

    y = omiga2_ML * y_1 + b2_ML;
    for(int i = 0; i < 32; i++)
    {
        y(i,0) = tanh(y(i,0));
    }

    Eigen::Matrix<double, 2,32> A_hat;
    for(int i =0; i< 32; i++)
    {
        A_hat(0,i) = e_v_ML[frame][0] * y(i,0);
        A_hat(1,i) = e_v_ML[frame][1] * y(i,0);
    }

    A_dot_ML = Tau_ML * (-A_hat);
    A_ML = A_dot_ML * T + A_dot_ML;
    f_e_ML.push_back(A_ML * y);
}

void controller_2::setMode(Mode mode)
{
    _mode = mode;
}

void controller_2::generateJointTrajectory()
{
    // cout << "generate tragectory !" << endl;
    sleep(2);
    unsigned int time_count = 0;

    Eigen::Vector2d X_0_tem , X_0_dot_tem, X_0_ddot_tem;
    

    double t;
    for (time_count = 0; time_count < count_total; time_count++ )
    {

        t = time_count * TimeUnit;
        if(time_count < 10501)
        {
            X_0_tem[0] = x_amplitude * t + init_pos[0];
            X_0_tem[1] = amplitude * sin(omiga * t) + init_pos[1];

            X_0_dot_tem[0] = x_amplitude;
            X_0_dot_tem[1] = 1.0 * amplitude * omiga * cos(omiga * t);

            X_0_ddot_tem[0] = 0.;
            X_0_ddot_tem[1] = - 1.0 * amplitude * omiga * omiga * sin(omiga * t);

            // X_0_tem[0] = 0.1 * sin(omiga * t) + init_pos[0];
            // X_0_tem[1] = 0 * amplitude * sin(omiga * t) + init_pos[1];
        
            // X_0_dot_tem[0] = 0.1 * omiga * cos(omiga * t);
            // X_0_dot_tem[1] = 0.;

            // X_0_ddot_tem[0] = -0.1 * omiga * omiga * sin(omiga * t);
            // X_0_ddot_tem[1] = 0.;
        }
        else
        {
            X_0_tem[0] = X0[10500][0];
            X_0_tem[1] = X0[10500][1];
        }

        // update x_0, x_0_dot, x_0_ddot

        X0.push_back(X_0_tem);
        X0_dot.push_back(X_0_dot_tem);
        X0_ddot.push_back(X_0_ddot_tem);

        model.getJacobianMatrixTwoDOF(q0[time_count], jacobian_now);

        if ( frame == 0)
        {
            jacobian_last = jacobian_now;
        }

        /// make sure that k, k+1, k-1
        /// update q0, q0_dot, q0_ddot

        q0_dot.push_back(jacobian_now.inverse() * X0_dot[time_count + 1]);
        q0.push_back(q0[time_count] + TimeUnit * q0_dot[time_count]);
        q0_ddot.push_back(jacobian_now.inverse() * (X0_ddot[time_count + 1] - (jacobian_now - jacobian_last) / TimeUnit * q0_dot[time_count]));

        jacobian_last = jacobian_now;
    }
}
// void controller_2::generateJointTrajectory()
// {
//     // cout << "generate tragectory !" << endl;
//     sleep(2);
//     unsigned int time_count = 0;

//     Eigen::Vector2d X_0_tem , X_0_dot_tem, X_0_ddot_tem;
    

//     double t;
//     for (time_count = 0; time_count < count_total; time_count++ )
//     {

//         t = time_count * TimeUnit;
//         if(time_count < 5260)
//         {
//             X_0_tem[0] = 0.005 * t + init_pos[0];
//             X_0_tem[1] = amplitude * sin(omiga *2 * t) + init_pos[1];

//             X_0_dot_tem[0] = 0.005;
//             X_0_dot_tem[1] = amplitude *  omiga *2 * cos(omiga *2 * t);

//             X_0_ddot_tem[0] = 0.;
//             X_0_ddot_tem[1] = - amplitude * omiga *2 * omiga *2 * sin(omiga *2 * t);
//         }
//         else
//         {
//             X_0_tem[0] = X0[5260][0];
//             X_0_tem[1] = X0[5260][1];
//         }

//         // update x_0, x_0_dot, x_0_ddot

//         X0.push_back(X_0_tem);
//         X0_dot.push_back(X_0_dot_tem);
//         X0_ddot.push_back(X_0_ddot_tem);

//         model.getJacobianMatrixTwoDOF(q0[time_count], jacobian_now);

//         if ( frame == 0)
//         {
//             jacobian_last = jacobian_now;
//         }

//         /// make sure that k, k+1, k-1
//         /// update q0, q0_dot, q0_ddot

//         q0_dot.push_back(jacobian_now.inverse() * X0_dot[time_count + 1]);
//         q0.push_back(q0[time_count] + TimeUnit * q0_dot[time_count]);
//         q0_ddot.push_back(jacobian_now.inverse() * (X0_ddot[time_count + 1] - (jacobian_now - jacobian_last) / TimeUnit * q0_dot[time_count]));

//         jacobian_last = jacobian_now;
//     }
// }

void controller_2::generateJointTrajectory_line()
{
    // cout << "generate tragectory !" << endl;
    sleep(2);
    unsigned int time_count = 0;

    Eigen::Vector2d X_0_tem , X_0_dot_tem, X_0_ddot_tem;
    

    double t;
    for (time_count = 0; time_count < count_total; time_count++ )
    {

        t = time_count * TimeUnit;
        if(time_count < 2500)
        {
            X_0_tem[0] = -0.02*t + init_pos[0];
            X_0_tem[1] = -0.05*t + init_pos[1];

            X_0_dot_tem[0] = -0.02;
            X_0_dot_tem[1] = -0.05;

            X_0_ddot_tem[0] = 0.;
            X_0_ddot_tem[1] = 0.;
        }
        else
        {
            X_0_tem[0] = X0[2500][0];
            X_0_tem[1] = X0[2500][1];
        }

        // update x_0, x_0_dot, x_0_ddot

        X0.push_back(X_0_tem);
        X0_dot.push_back(X_0_dot_tem);
        X0_ddot.push_back(X_0_ddot_tem);

        model.getJacobianMatrixTwoDOF(q0[time_count], jacobian_now);

        if ( frame == 0)
        {
            jacobian_last = jacobian_now;
        }

        /// make sure that k, k+1, k-1
        /// update q0, q0_dot, q0_ddot

        q0_dot.push_back(jacobian_now.inverse() * X0_dot[time_count + 1]);
        q0.push_back(q0[time_count] + TimeUnit * q0_dot[time_count]);
        q0_ddot.push_back(jacobian_now.inverse() * (X0_ddot[time_count + 1] - (jacobian_now - jacobian_last) / TimeUnit * q0_dot[time_count]));

        jacobian_last = jacobian_now;
    }
}

Eigen::Vector2d controller_2::getTorque(const double & T, Eigen::Vector2d& f_from_sensor, Eigen::Vector2d& q_from_robot)
{
    Eigen::Vector2d tor;
    switch (_mode)
    {
    case Mode::NORMAL:
        tor = getTorqueNormal(T, f_from_sensor, q_from_robot);
        return tor;
        break;

    case Mode::DI:
        tor = getTorqueDI(T, f_from_sensor, q_from_robot);
        return tor;
        break;

    case Mode::DISM:
        tor = getTorqueDISM(T, f_from_sensor, q_from_robot);
        return tor;
        break;

    case Mode::DIML:
        tor = getTorqueDIML(T, f_from_sensor, q_from_robot);
        return tor;
        break;
        
    default:
        cout << "please choose a control mode for the robot!" << endl;
        break;
    }
}


Eigen::Vector2d controller_2::getTorqueDI(const double & T, Eigen::Vector2d& f_from_sensor, Eigen::Vector2d& q_from_robot)
{
    K_hat = K + B / T + L * T;
    model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
    f_ext.push_back(f_from_sensor);
    //update q and f_ext , get them from sensor
    q.push_back(q_from_robot);
    tau_ext.push_back(jacobian_now.transpose() * f_from_sensor);


    // get new u_x_star
    Eigen::Vector2d u_x_star_tem = (M_d_a + D_d_a * T).inverse() * 
    (M_d_a * u_x[frame] + T * (M_d_a * q0_ddot[frame + 1] + D_d_a * q0_dot[frame + 1] + K_d_a * q0[frame + 1] + tau_ext[frame + 1]));
    u_x_star.push_back(u_x_star_tem);
    
    // get new q_x_star
    Eigen::Vector2d q_x_star_tem = q_x[frame] + T * u_x_star[frame + 1];
    q_x_star.push_back(q_x_star_tem);

    // get new phi_b
    Eigen::Vector2d phi_b_tem;
    phi_b_tem = (B * (q_x[frame] - q[frame])) / T - L * a[frame];
    phi_b.push_back(phi_b_tem);

    // get new phi a
    Eigen::Vector2d phi_a_tem;
    phi_a_tem = M * (q[frame + 1] - q_x[frame] - T * u_x[frame]) / (T * T);
    phi_a.push_back(phi_a_tem);

    // get new q_star
    Eigen::Vector2d q_star_tem;
    q_star_tem = q[frame + 1] + (K_hat + M / (T * T)).inverse() * (phi_b[frame + 1] - phi_a[frame + 1]);
    q_star.push_back(q_star_tem);

    cout << "q_star_tem:" <<  q_star[frame + 1].transpose() << endl;


    // get mat1 and mat2
    Eigen::Matrix2d Mat1  = Eigen::Matrix2d::Identity() + (M_d_a + D_d_a * T).inverse() * K_d_a * (T * T);
    Eigen::Matrix2d Mat2 = K_hat + M / (T * T);

    // get new tau_star
    Eigen::Vector2d tau_star_tem;
    tau_star_tem = Mat2 * Mat1.inverse() * q_x_star[frame + 1] - Mat2 * q_star[frame + 1];
    tau_star.push_back(tau_star_tem);

    
    cout << "tau_star:" <<  tau_star[frame + 1].transpose() << endl;

    // get new output tau
    Eigen::Vector2d tau_tem;
    tau_tem = proj(tau_star_tem);
    tau.push_back(tau_tem);

    cout << "frame: "  << frame << endl;
    return tau[frame + 1];

}


Eigen::Vector2d controller_2::getTorqueNormal_2(const double & T, Eigen::Vector2d& f_from_sensor, Eigen::Vector2d& q_from_robot)
{
    model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
    f_ext.push_back(f_from_sensor);
    //update q and f_ext , get them from sensor
    q.push_back(q_from_robot);
    tau_ext.push_back(jacobian_now.transpose() * f_from_sensor);
    // f.push_back(Eigen::Vector2d::Zero());


    Eigen::Vector2d q_x_tem;
    if (frame == 0)
    {
        q_x_tem = (M_d_a / (T * T) + D_d_a / T + K_d_a).inverse()
        * (M_d_a / (T * T) * (2 * q_x[frame])
        + D_d_a / T * q_x[frame] + (M_d_a * q0_ddot[frame + 1] + D_d_a * q0_dot[frame + 1] + K_d_a * q0[frame + 1] + tau_ext[frame + 1]));
    }else{
        q_x_tem = (M_d_a / (T * T) + D_d_a / T + K_d_a).inverse()
        * (M_d_a / (T * T) * (2 * q_x[frame] - q_x[frame - 1])
        + D_d_a / T * q_x[frame] + (M_d_a * q0_ddot[frame + 1] + D_d_a * q0_dot[frame + 1] + K_d_a * q0[frame + 1] + tau_ext[frame + 1]));
    }
    q_x.push_back(q_x_tem);

    // get new u_x_star
    Eigen::Vector2d u_x_tem = (q_x[frame + 1] - q_x[frame]) / T;
    u_x.push_back(u_x_tem);
    
    
    Eigen::Vector2d a_tem;
    a_tem = a[frame] + T * (q_x[frame + 1] - q[frame + 1]);
    a.push_back(a_tem);

    Eigen::Vector2d tau_tem;
    if (frame == 0)
    {
        tau_tem = M * (q_x[frame + 1] - 2 * q_x[frame]) / (T * T)
        + B * (T * u_x[frame + 1] - (q[frame + 1] - q[frame])) / T
        + K * (q_x[frame + 1] - q[frame + 1]) + L * a[frame + 1];
    }else 
    {
        tau_tem = M * (q_x[frame + 1] - 2 * q_x[frame] + q_x[frame - 1]) / (T * T)
        + B * (T * u_x[frame + 1] - (q[frame + 1] - q[frame])) / T
        + K * (q_x[frame + 1] - q[frame + 1]) + L * a[frame + 1];
    }

    Eigen::Vector2d tau_pro;
    cout << "tau_star:" <<  tau_tem.transpose() << endl;
    tau_pro = proj(tau_tem);
    tau.push_back(tau_pro);

    return tau[frame + 1];

}

Eigen::Vector2d controller_2::getTorqueNormal(const double & T, Eigen::Vector2d& f_from_sensor, Eigen::Vector2d& q_from_robot)
{
    model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
    f_ext.push_back(f_from_sensor);
    //update q and f_ext , get them from sensor
    q.push_back(q_from_robot);
    tau_ext.push_back(jacobian_now.transpose() * f_from_sensor);
    // f.push_back(Eigen::Vector2d::Zero());

    Eigen::Vector2d q_x_ddot;
    q_x_ddot = M_d_a.inverse() * 
    (-D_d_a * (q_x_dot[frame] -q0_dot[frame + 1]) - K_d_a * (q_x[frame] - q0[frame + 1]) + tau_ext[frame + 1]) + q0_ddot[frame +1];

    Eigen::Vector2d q_x_dot_tem;
    q_x_dot_tem = q_x_dot[frame] + T * q_x_ddot;
    q_x_dot.push_back(q_x_dot_tem);

    Eigen::Vector2d q_x_tem;
    q_x_tem = q_x[frame] + T * q_x_dot[frame + 1];
    q_x.push_back(q_x_tem);

    Eigen::Vector2d a_tem;
    a_tem = a[frame] + T * (q_x[frame + 1] - q[frame + 1]);
    a.push_back(a_tem);

    Eigen::Vector2d q_dot;
    q_dot = (q[frame + 1] - q[frame]) / T;

    Eigen::Vector2d tau_star_tem;
    tau_star_tem = M * q_x_ddot + K * (q_x[frame + 1] - q[frame + 1]) + B * (q_x_dot[frame + 1] - q_dot) + L * a[frame + 1];
    tau_star.push_back(tau_star_tem);

    cout << "tau_star:" <<  tau_star_tem.transpose() << endl;

    Eigen::Vector2d tau_tem;
    tau_tem = proj(tau_star_tem);
    tau.push_back(tau_tem);

    return tau_tem;


}

// Eigen::Vector2d controller_2::getTorqueDISM(const double & T, Eigen::Vector2d& f_from_sensor, Eigen::Vector2d& q_from_robot)
// {
    
//     double h = T;
//     model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
//     f_ext.push_back(f_from_sensor);
//     //update q and f_ext , get them from sensor
//     q.push_back(q_from_robot);
//     tau_ext.push_back(jacobian_now.transpose() * f_from_sensor);
//     // tau_ext.push_back(Eigen::Vector2d::Zero());

//     Eigen::Matrix2d h_x;
//     h_x = (M_d_a + h * D_d_a).inverse() * h;

//     Eigen::Matrix2d h_s;
//     h_s = (M_s + h * D_s).inverse() * h;

//     Eigen::Matrix2d lamda_1;
//     lamda_1 = (Eigen::Matrix2d::Identity() + h * Lamda_1).inverse();

//     Eigen::Matrix2d lamda_2;
//     lamda_2 = (Eigen::Matrix2d::Identity() + h * Lamda_2).inverse();

//     Eigen::Vector2d set;
//     set = -tau_ext[frame + 1] - (M_d_a * q0_ddot[frame + 1] + D_d_a * q0_dot[frame + 1] + K_d_a * q0[frame + 1]);

//     Eigen::Matrix2d h_x_star;
//     h_x_star = (Eigen::Matrix2d::Identity() + h * h_x * K_d_a).inverse();

//     Eigen::Vector2d q1;
//     q1 = lamda_1 * (q_x[frame] + h* Lamda_1 * q[frame + 1] + h_s * M_s * e_r[frame]);

//     Eigen::Vector2d q2;
//     q2 = h_x_star * q_x[frame] + h_x_star * h_x * (M_d_a * v_x[frame] - h * set);

//     Eigen::Vector2d q3;
//     q3 = q[frame + 1] + lamda_2 * e_q[frame];

//     Eigen::Matrix2d S1;
//     S1.block(0,0,2,1) = (lamda_1 * h_s * h * (F_max - F0));
//     S1.block(0,1,2,1) = (lamda_1 * h_s * h * (F_max + F0));

//     Eigen::Matrix2d _S1;
//     _S1.block(0,0,2,1) = - (lamda_1 * h_s * h * (F_max + F0));
//     _S1.block(0,1,2,1) = - (lamda_1 * h_s * h * (F_max - F0));

//     Eigen::Matrix2d S0;
//     S0.block(0,0,2,1) = project(_S1, q3 - q1);
//     S0.block(0,1,2,1) = project(S1, q3 - q1);

//     q_x.push_back(q1 + project(S0, q2 - q1));

//     Eigen::Vector2d q4;
//     if (frame == 0)
//     {
//         q4 = q[frame] + h_s * h * tau_ext[frame + 1];
//     }
//     else
//     {

//         q4 = q[frame] + h_s * M_s * (q[frame] - q[frame -1]) / h + h_s * h * tau_ext[frame + 1];
//     }

//     Eigen::Vector2d q5;
//     q5 = (h_s * h).inverse() * (q_x[frame + 1] - lamda_2 * e_q[frame] - q4);

//     Eigen::Vector2d q6;
//     q6 = (lamda_1 * h_s * h).inverse() * (q_x[frame + 1] - q1);

//     Eigen::Matrix2d F_h;
//     F_h.block(0,0,2,1) = -F0;
//     F_h.block(0,1,2,1) = F0;

//     Eigen::Vector2d tau_tem;
//     tau_tem = q6 + project(F_h, q5 - q6);
//     tau.push_back(tau_tem);

//     return tau[frame + 1];
// }

// AdmDISMCPro
Eigen::Vector2d controller_2::getTorqueDISM(const double& T, Eigen::Vector2d& f_from_sensor, Eigen::Vector2d& q_from_robot)
{

    double h = T;
    // f_from_sensor[0] = 0.;
    model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
    f_ext.push_back(f_from_sensor);
    //update q and f_ext , get them from sensor
    q.push_back(q_from_robot);
    tau_ext.push_back(jacobian_now.transpose() * f_from_sensor);
    // tau_ext.push_back(Eigen::Vector2d::Zero());

    Eigen::Matrix2d h_x;
    h_x = (M_d_a + h * D_d_a).inverse() * h;

    Eigen::Matrix2d h_s;
    h_s = (M_s + h * D_s).inverse() * h;

    Eigen::Matrix2d lamda_1;
    lamda_1 = (Eigen::Matrix2d::Identity() + h * Lamda_1).inverse();

    Eigen::Matrix2d lamda_2;
    lamda_2 = (Eigen::Matrix2d::Identity() + h * Lamda_2).inverse();

    Eigen::Vector2d set;
    set = -tau_ext[frame + 1] - (M_d_a * q0_ddot[frame + 1] + D_d_a * q0_dot[frame + 1] + K_d_a * q0[frame + 1]);

    Eigen::Matrix2d h_x_star;
    h_x_star = (Eigen::Matrix2d::Identity() + h * h_x * K_d_a).inverse();

    Eigen::Matrix2d h_q;
    h_q = (M_s_hat + h * D_s_hat).inverse() * h;

    Eigen::Vector2d q1;
    q1 = lamda_1 * (q_x[frame] + h * Lamda_1 * q[frame + 1] + h_s * M_s * e_r[frame]);

    Eigen::Vector2d q2;
    q2 = h_x_star * q_x[frame] + h_x_star * h_x * (M_d_a * v_x[frame] - h * set);

    Eigen::Vector2d q3;
    q3 = q[frame + 1] + lamda_2 * e_q[frame];

    Eigen::Matrix2d S1;
    S1.block(0, 0, 2, 1) = (lamda_1 * h_s * h * (F_max - F0));
    S1.block(0, 1, 2, 1) = (lamda_1 * h_s * h * (F_max + F0));

    Eigen::Matrix2d _S1;
    _S1.block(0, 0, 2, 1) = -(lamda_1 * h_s * h * (F_max + F0));
    _S1.block(0, 1, 2, 1) = -(lamda_1 * h_s * h * (F_max - F0));

    Eigen::Matrix2d S0;
    S0.block(0, 0, 2, 1) = project(_S1, q3 - q1);
    S0.block(0, 1, 2, 1) = project(S1, q3 - q1);

    q_x.push_back(q1 + project(S0, q2 - q1));

    Eigen::Vector2d q4;
    if (frame == 0)
    {
        q4 = q[frame] + h_q * h * tau_ext[frame + 1];
    }
    else
    {

        q4 = q[frame] + h_q * M_s_hat * (q[frame] - q[frame - 1]) / h + h_q * h * tau_ext[frame + 1];
    }

    Eigen::Vector2d q5;
    q5 = (h_q * h).inverse() * (q_x[frame + 1] - lamda_2 * e_q[frame] - q4);

    Eigen::Vector2d q6;
    q6 = (lamda_1 * h_s * h).inverse() * (q_x[frame + 1] - q1);

    Eigen::Matrix2d F_h;
    F_h.block(0, 0, 2, 1) = -F0;
    F_h.block(0, 1, 2, 1) = F0;


    Eigen::Vector2d tau_tem;
    tau_tem = q6 + project(F_h, q5 - q6);
    

    Eigen::Vector2d tau_dot;
    tau_dot = - 0. * (tau_tem - tau[frame]) / 10.0;

    Eigen::Vector2d tau_out;
    tau_out = tau_dot + tau_tem;
    tau_out = proj(tau_out);
    tau.push_back(tau_out);
    return tau[frame+1];
    
}

// AdmDISMCPro+PD
// Eigen::Vector2d controller_2::getTorqueDISM(const double& T, Eigen::Vector2d& f_from_sensor, Eigen::Vector2d& q_from_robot)
// {

//    double h = T;
//    model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
//    f_ext.push_back(f_from_sensor);
//    //update q and f_ext , get them from sensor
//    q.push_back(q_from_robot);
//    tau_ext.push_back(jacobian_now.transpose() * f_from_sensor);
//    // tau_ext.push_back(Eigen::Vector2d::Zero());

//    Eigen::Matrix2d h_x;
//    h_x = (M_d_a + h * D_d_a).inverse() * h;

//    Eigen::Matrix2d h_s;
//    h_s = (M_s + h * D_s).inverse() * h;

//    Eigen::Matrix2d lamda_1;
//    lamda_1 = (Eigen::Matrix2d::Identity() + h * Lamda_1).inverse();

//    Eigen::Matrix2d lamda_2;
//    lamda_2 = (Eigen::Matrix2d::Identity() + h * Lamda_2).inverse();

//    Eigen::Vector2d set;
//    set = -tau_ext[frame + 1] - (M_d_a * q0_ddot[frame + 1] + D_d_a * q0_dot[frame + 1] + K_d_a * q0[frame + 1]);

//    Eigen::Matrix2d h_x_star;
//    h_x_star = (Eigen::Matrix2d::Identity() + h * h_x * K_d_a).inverse();

//    Eigen::Matrix2d h_q;
//    h_q = (M_s_hat + h * D_s_hat).inverse() * h;

//    Eigen::Matrix2d lamda_1_star;
//    lamda_1_star = (Eigen::Matrix2d::Identity() + lamda_1 * h_s * K_star * (Eigen::Matrix2d::Identity() + h * Lamda_2)).inverse();

//    Eigen::Vector2d q1;
//    q1 = lamda_1_star * lamda_1 * (q_x[frame] + (h * Lamda_1 + h_s * K_star * (Eigen::Matrix2d::Identity() + h * Lamda_2)) * q[frame + 1] + h_s * M_s * e_r[frame] + h_s * K_star * e_q[frame]);

//    Eigen::Vector2d q2;
//    q2 = h_x_star * q_x[frame] + h_x_star * h_x * (M_d_a * v_x[frame] - h * set);

//    Eigen::Vector2d q3;
//    q3 = q[frame + 1] + lamda_2 * e_q[frame];

//    Eigen::Matrix2d S1;
//    S1.block(0, 0, 2, 1) = (lamda_1_star * lamda_1 * h_s * h * (F_max - F0));
//    S1.block(0, 1, 2, 1) = (lamda_1_star * lamda_1 * h_s * h * (F_max + F0));

//    Eigen::Matrix2d _S1;
//    _S1.block(0, 0, 2, 1) = -(lamda_1_star * lamda_1 * h_s * h * (F_max + F0));
//    _S1.block(0, 1, 2, 1) = -(lamda_1_star * lamda_1 * h_s * h * (F_max - F0));

//    Eigen::Matrix2d S0;
//    S0.block(0, 0, 2, 1) = project(_S1, q3 - q1);
//    S0.block(0, 1, 2, 1) = project(S1, q3 - q1);

//    q_x.push_back(q1 + project(S0, q2 - q1));

//    Eigen::Vector2d q4;
//    if (frame == 0)
//    {
//        q4 = q[frame] + h_q * h * tau_ext[frame + 1];
//    }
//    else
//    {

//        q4 = q[frame] + h_q * M_s_hat * (q[frame] - q[frame - 1]) / h + h_q * h * tau_ext[frame + 1];
//    }

//    Eigen::Vector2d q5;
//    q5 = (h_q * h).inverse() * (q_x[frame + 1] - lamda_2 * e_q[frame] - q4);

//    Eigen::Vector2d q6;
//    q6 = (lamda_1_star * lamda_1 * h_s * h).inverse() * (q_x[frame + 1] - q1);

//    Eigen::Matrix2d F_h;
//    F_h.block(0, 0, 2, 1) = -F0;
//    F_h.block(0, 1, 2, 1) = F0;

//    Eigen::Vector2d tau_tem;
//    tau_tem = q6 + project(F_h, q5 - q6);
//    tau.push_back(tau_tem);

//    return tau[frame + 1];
// }

// Eigen::Vector2d controller_2::getTorqueDIML(const double &T, Eigen::Vector2d &f_ext_from_sensor, Eigen::Vector2d &q_from_robot)
// {
//     model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
//     f_ext.push_back(f_ext_from_sensor);
//     //update q and f_ext , get them from sensor
//     q.push_back(q_from_robot);
//     tau_ext.push_back(jacobian_now.transpose() * f_ext_from_sensor);


//     h_s_ML = (M_s_ML + T * D_s_ML) / T;
//     lamda1_ML = (Eigen::Matrix2d::Identity() + T * Lamda1_ML) /T;
//     lamda2_ML = (Eigen::Matrix2d::Identity() + T * Lamda2_ML) /T;
//     h_1_ML = Eigen::Matrix2d::Identity() + (M_x_ML +D_x_ML * T).inverse() * K_x_ML * T * T;

//     Eigen::Vector2d u_x_star_ML;
//     u_x_star_ML= (M_x_ML + D_x_ML * T).inverse()*(M_x_ML * u_x_ML[frame] + T * (M_x_ML * q0_ddot[frame + 1]) + D_x_ML * q0_dot[frame + 1] + K_x_ML * q0[frame + 1] + tau_ext[frame + 1]);
//     Eigen::Vector2d q_x_star_ML;
//     q_x_star_ML = q_x[frame] + T * u_x_star_ML;


//     // Eigen::Matrix<double, 32,1> y;
//     // Eigen::Matrix<double, 32,1> y_1;
//     // Eigen::Vector4d x;
//     // if(frame == 0)
//     // {
//     //     x << q[frame][0], q[frame][1], q[frame][0]  / T , q[frame ][1] / T;

//     // }else{

//     //     x << q[frame][0], q[frame][1], (q[frame][0] - q[frame-1][0]) / T , (q[frame ][1] - q[frame-1][1]) / T;
//     // }

//     // y_1 = omiga1_ML*x + b1_ML;
//     // for(int i = 0; i < 32; i++)
//     // {
//     //     y_1(i,0) = tanh(y_1(i,0));
//     // }

//     // y = omiga2_ML * y_1 + b2_ML;
//     // for(int i = 0; i < 32; i++)
//     // {
//     //     y(i,0) = tanh(y(i,0));
//     // }

//     // Eigen::Matrix<double, 2,32> A_hat;
//     // for(int i =0; i< 32; i++)
//     // {
//     //     A_hat(0,i) = e_v_ML[frame][0] * y(i,0);
//     //     A_hat(1,i) = e_v_ML[frame][1] * y(i,0);
//     // }

//     // A_dot_ML = Tau_ML * A_hat;
//     // A_ML = A_dot_ML * T + A_dot_ML;
//     // f_e_ML.push_back(A_ML * y);
//     f_e_ML.push_back(Eigen::Vector2d::Zero());

    

//     Eigen::Vector2d c_1_ML_tem;
//     // c_1_ML_tem = - (h_s_ML * Lamda1_ML + K_ML * lamda2_ML)*q[frame+1] - (h_s_ML*q_x[frame]/T + K_ML*e_q_ML[frame]/T);
//     c_1_ML_tem = - (h_s_ML * Lamda1_ML + K_ML * lamda2_ML)*q[frame+1] - (h_s_ML*q_x[frame]/T + M_s_ML*e_r_ML[frame]/T  + K_ML*e_q_ML[frame]/T);
//     c_1_ML.push_back(c_1_ML_tem);


//     Eigen::Vector2d tau_star_tem;
//     tau_star_tem = (h_s_ML * lamda1_ML + K_ML * lamda2_ML)*(h_1_ML.inverse())*q_x_star_ML + c_1_ML[frame + 1];
//     tau_star.push_back(tau_star_tem);
//     Eigen::Vector2d tau_tem;
//     tau_tem = proj(tau_star_tem);

//     tau.push_back(tau_tem);
//     return tau[frame + 1];
// }

Eigen::Vector2d controller_2::getTorqueDIML(const double &T, Eigen::Vector2d &f_from_sensor, Eigen::Vector2d &q_from_robot)
{
    double h = T;
    // f_from_sensor[0] = 0.;
    model.getJacobianMatrixTwoDOF(q_from_robot, jacobian_now);
    f_ext.push_back(f_from_sensor);
    //update q and f_ext , get them from sensor
    q.push_back(q_from_robot);
    tau_ext.push_back(jacobian_now.transpose() * f_from_sensor);
    // tau_ext.push_back(Eigen::Vetor2d::Zero());

    Eigen::Vector2d adp_term;
    // adp_term = 1.0 * f_e_ML[frame];
    adp_term = 1. * f_e_ML[frame];

    // if(tau_ext[frame + 1][0] < -0.4) 
    // {
    //     adp_term = 0. * f_e_ML[frame];
    // }
    // else
    // {
    //     adp_term = 1. * f_e_ML[frame];
    // }

    Eigen::Matrix2d h_x;
    h_x = (M_d_a + h * D_d_a).inverse() * h;

    Eigen::Matrix2d h_s;
    h_s = (M_s + h * D_s).inverse() * h;

    Eigen::Matrix2d lamda_1;
    lamda_1 = (Eigen::Matrix2d::Identity() + h * Lamda_1).inverse();

    Eigen::Matrix2d lamda_2;
    lamda_2 = (Eigen::Matrix2d::Identity() + h * Lamda_2).inverse();

    Eigen::Vector2d set;
    set = -tau_ext[frame + 1] - (M_d_a * q0_ddot[frame + 1] + D_d_a * q0_dot[frame + 1] + K_d_a * q0[frame + 1]);

    Eigen::Matrix2d h_x_star;
    h_x_star = (Eigen::Matrix2d::Identity() + h * h_x * K_d_a).inverse();

    Eigen::Matrix2d h_q;
    h_q = (M_s_hat + h * D_s_hat).inverse() * h;

    Eigen::Vector2d q1;
    q1 = lamda_1 * (q_x[frame] + h * Lamda_1 * q[frame + 1] + h_s * M_s * e_r[frame] + h_s * h * adp_term);
    Eigen::Vector2d q2;
    q2 = h_x_star * q_x[frame] + h_x_star * h_x * (M_d_a * v_x[frame] - h * set);

    Eigen::Vector2d q3;
    q3 = q[frame + 1] + lamda_2 * e_q[frame];


    Eigen::Matrix2d S1;
    S1.block(0, 0, 2, 1) = (lamda_1 * h_s * h * (F_max - F0));
    S1.block(0, 1, 2, 1) = (lamda_1 * h_s * h * (F_max + F0));

    Eigen::Matrix2d _S1;
    _S1.block(0, 0, 2, 1) = -(lamda_1 * h_s * h * (F_max + F0));
    _S1.block(0, 1, 2, 1) = -(lamda_1 * h_s * h * (F_max - F0));

    Eigen::Matrix2d S0;
    S0.block(0, 0, 2, 1) = project(_S1, q3 - q1);
    S0.block(0, 1, 2, 1) = project(S1, q3 - q1);

    q_x.push_back(q1 + project(S0, q2 - q1));

    Eigen::Vector2d q4;
    if (frame == 0)
    {
        q4 = q[frame] + h_q * h * (tau_ext[frame + 1] + adp_term);
    }
    else
    {

        q4 = q[frame] + h_q * M_s_hat * (q[frame] - q[frame - 1]) / h + h_q * h * (tau_ext[frame + 1] + adp_term);
    }

    Eigen::Vector2d q5;
    q5 = (h_q * h).inverse() * (q_x[frame + 1] - lamda_2 * e_q[frame] - q4);

    Eigen::Vector2d q6;
    q6 = (lamda_1 * h_s * h).inverse() * (q_x[frame + 1] - q1);

    Eigen::Matrix2d F_h;
    F_h.block(0, 0, 2, 1) = -F0;
    F_h.block(0, 1, 2, 1) = F0;


    Eigen::Vector2d tau_tem;
    tau_tem = q6 + project(F_h, q5 - q6);
    
    tau.push_back(tau_tem);

    return tau[frame+1];
}

void controller_2::refresh(const double & T)
{
    switch (_mode)
    {
    case Mode::DI:
        refreshDI(T);
        break;
    case Mode::NORMAL:
        refreshNormal(T);
        break;
    case Mode::DISM:
        refreshDISM(T);
        break;
    case Mode::DIML:
        refreshDIML(T);
        break;
    default:
        cout << "please choose a control mode for the robot!" << endl;
        break;
    }
}

void controller_2::refreshDI(const double & T)
{

    K_hat = K + B / T + L * T;

    // get new q_x
    Eigen::Vector2d q_x_tem;
    q_x_tem = q_star[frame + 1] + (K_hat + M / (T * T)).inverse() * tau[frame + 1];
    q_x.push_back(q_x_tem);

    //get new u_x
    Eigen::Vector2d u_x_tem;
    u_x_tem = (q_x[frame + 1] - q_x[frame]) / T;
    u_x.push_back(u_x_tem);

    // get new a
    Eigen::Vector2d a_tem;
    a_tem = a[frame] + T * (q_x[frame + 1] - q[frame + 1]);
    a.push_back(a_tem);

    frame += 1;
}

void controller_2::refreshNormal(const double & T)
{

    frame += 1;
}

void controller_2::refreshDISM(const double & T)
{
    Eigen::Vector2d v_x_tem;
    v_x_tem = (q_x[frame + 1] - q_x[frame]) / T;
    v_x.push_back(v_x_tem);

    Eigen::Vector2d e_q_tem;
    e_q_tem = q_x[frame + 1] - q[frame + 1];
    e_q.push_back(e_q_tem);

    Eigen::Vector2d e_r_tem;
    e_r_tem = v_x[frame + 1] + Lamda_1 * e_q[frame + 1];
    e_r.push_back(e_r_tem);

    frame += 1;
}

void controller_2::refreshDIML(const double & T)
{

    Eigen::Vector2d v_x_tem;
    v_x_tem = (q_x[frame + 1] - q_x[frame]) / T;
    v_x.push_back(v_x_tem);

    Eigen::Vector2d e_q_tem;
    e_q_tem = q_x[frame + 1] - q[frame + 1];
    e_q.push_back(e_q_tem);

    Eigen::Vector2d e_r_tem;
    e_r_tem = v_x[frame + 1] + Lamda_1 * e_q[frame + 1];
    e_r.push_back(e_r_tem);

    Eigen::Vector2d e_v_ML_tem;
    e_v_ML_tem = (e_q[frame + 1] - e_q[frame]) / T + Lamda_2 * e_q[frame + 1];
    e_v_ML.push_back(-e_v_ML_tem);


    Eigen::Matrix<double, 32,1> y;
    Eigen::Matrix<double, 32,1> y_1;
    Eigen::Vector4d x;
    
    x << q[frame+1][0], q[frame+1][1], (q[frame+1][0] - q[frame][0]) / T , (q[frame + 1][1] - q[frame][1]) / T;

    y_1 = omiga1_ML*x + b1_ML;
    for(int i = 0; i < 32; i++)
    {
        y_1[i] = tanh(y_1[i]);
    }


    y = omiga2_ML * y_1 + b2_ML;
    for(int i = 0; i < 32; i++)
    {
        y[i] = tanh(y[i]);
    }


    Eigen::Matrix<double, 2,32> A_hat;
    for(int i =0; i< 32; i++)
    {
        A_hat(0,i) = e_v_ML[frame+1][0] * y[i];
        A_hat(1,i) = e_v_ML[frame+1][1] * y[i];
    }

    A_dot_ML = Tau_ML * A_hat;

    A_ML = A_dot_ML * T + A_ML;

    f_e_ML.push_back(A_ML * y);

    frame += 1;
}


Eigen::Vector2d controller_2::proj(Eigen::Vector2d& tau_star_input)
{
    Eigen::Vector2d tau_projected;
    for (int i = 0; i < 2; i++)
    {
        if (tau_star_input[i] < -F_max[i])
        {
            tau_projected[i] =  -F_max[i];
        }
        else if (tau_star_input[i] > F_max[i])
        {
            tau_projected[i] = F_max[i];
        }
        else
        {
            tau_projected[i] = tau_star_input[i];
        }
    }
    return tau_projected;
}

Eigen::Vector2d controller_2::project(Eigen::Matrix2d S, Eigen::Vector2d q)
{
    Eigen::Vector2d q_output;
    for (int i = 0; i < 2; i ++)
    {
        if (q[i] < S(i,0))
        {
            q_output[i] = S(i,0);
        }
        else if(q[i] > S(i,1))
        {
            q_output[i] = S(i,1);
        }else
        {
            q_output[i] = q[i];
        }
    }
    return q_output;
}

void controller_2::plotJointTorque()
{
    // plot tau_star
    vector<vector<double>> tau_star_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < tau_star.size(); i++)
        { 
            
            tem.push_back(tau_star[i][j]);
        }
        tau_star_plot.push_back(tem);
    }

    // plot tau
    Kalman myFilter(10,100,100,0);

    vector<vector<double>> tau_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i = 0; i < tau.size(); i++)
        {   
            // tau[i][0] = myFilter.getFilteredValue(tau[i][0]);
            if(tau[i][j] < -F_max[0] )
            {
                tau[i][j]  = -F_max[0];
            }else if(tau[i][j] > F_max[0])
            {
                tau[i][j]  = F_max[0];
            }
            tem.push_back(tau[i][j]);
        }
        tau_plot.push_back(tem);
    }
    
    plt::figure_size(1200, 780);
    for (int i = 0; i < 2; i++)
    {
        string tau_star_string, tau_string;

        tau_star_string = "tau_star" + to_string(i+1);
        tau_string = "tau" + to_string(i+1);

        plt::named_plot(tau_star_string, tau_star_plot[i], "--");
        plt::named_plot(tau_string, tau_plot[i]);
    }
    

    plt::xlabel("Time (us)");
    plt::ylabel("Torque (Nm)");
    plt::legend();
    plt::save("JointTorque.pdf");
}


void controller_2::plotExternalForce()
{
    // plot f_ext
    vector<vector<double>> tau_ext_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < tau_ext.size(); i++)
        {
            tem.push_back(tau_ext[i][j]);
        }
        tau_ext_plot.push_back(tem);
    }

    vector<vector<double>> f_ext_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < f_ext.size(); i++)
        {
            tem.push_back(f_ext[i][j]);
        }
        f_ext_plot.push_back(tem);
    }

    plt::figure_size(1200, 780);
    for (int i = 0; i < 2; i++)
    {
        string tau_ext_string, f_ext_string;
        tau_ext_string = "tau_ext" + to_string(i+1);
        f_ext_string = "f_ext" + to_string(i+1);
        // plt::named_plot(tau_ext_string, tau_ext_plot[i]);
        plt::named_plot(f_ext_string, f_ext_plot[i]);

    }

    plt::xlabel("Time (us)");
    plt::ylabel("Force (N)");
    plt::title("External Force");
    plt::legend();
    plt::save("ExternalForce.pdf");
}

void controller_2::plotExternalJointForce()
{
    // plot f_ext
    vector<vector<double>> tau_ext_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < tau_ext.size(); i++)
        {
            tem.push_back(tau_ext[i][j]);
        }
        tau_ext_plot.push_back(tem);
    }

    vector<vector<double>> f_e_ML_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < f_e_ML.size(); i++)
        {
            tem.push_back(f_e_ML[i][j]);
        }
        f_e_ML_plot.push_back(tem);
        // std::cout<<"f_e_ML.size(): "<<f_e_ML.size()<<std::endl;
    }

    plt::figure_size(1200, 780);
    for (int i = 0; i < 2; i++)
    {
        string tau_ext_string, f_e_ML_string;
        tau_ext_string = "tau_ext" + to_string(i+1);
        f_e_ML_string = "f_e_ML" + to_string(i+1);
        plt::named_plot(tau_ext_string, tau_ext_plot[i]);
        plt::named_plot(f_e_ML_string, f_e_ML_plot[i]);
    }
    plt::xlabel("Time (us)");
    plt::ylabel("Force (Nm)");
    plt::title("External Joint Force");
    plt::legend();
    plt::save("ExternalJointForce.pdf");
}


void controller_2::plotJointAngles()
{
    // plot q
    vector<vector<double>> q_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < q.size(); i++)
        {
            tem.push_back(q[i][j]);
        }
        q_plot.push_back(tem);
    }

    // plot q_x
    vector<vector<double>> q_x_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < q_x.size(); i++)
        {
            tem.push_back(q_x[i][j]);
        }
        q_x_plot.push_back(tem);
    }

    vector<vector<double>> q0_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i = 0; i < q0.size(); i++)
        {
            tem.push_back(q0[i][j]);
        }
        q0_plot.push_back(tem);
    }

    plt::figure_size(1200, 780);

    for (int i = 0; i < 2; i++)
    {
        string q_string, q_x_string, q_0_string;
        q_string = "q_joint" + to_string(i+1);
        q_x_string = "q_x_joint" + to_string(i+1);
        q_0_string = "q_0_joint" + to_string(i+1);

        plt::named_plot(q_string, q_plot[i]);
        plt::named_plot(q_x_string, q_x_plot[i],"--" );
        plt::named_plot(q_0_string, q0_plot[i]);
        
    }

    plt::legend();
    plt::xlabel("Time (us)");
    plt::ylabel("Position (rad)");
    plt::title("Joint Angles");
    // plt::xlim(0, 100);
    plt::save("JointAngles.pdf");
}


void controller_2::plotCartesianPosition()
{
    KortexKinematics model;
    // plot tau_star
    vector<vector<double>> X0_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < X0.size(); i++)
        {
            tem.push_back(X0[i][j]);
        }
        X0_plot.push_back(tem);
    }

    
    Eigen::Vector2d X_tem;
    for(size_t i = 0; i < q.size(); i ++)
    {
        model.getFowardKinematicsTwoDOF(q[i+1], X_tem);
        X.push_back(X_tem);
    }

    vector<vector<double>> X_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < X.size(); i++)
        {
            tem.push_back(X[i][j]);
        }
        X_plot.push_back(tem);
    }

    
    Eigen::Vector2d X_d_tem;
    for(size_t i = 0; i < q_x.size(); i ++)
    {
        model.getFowardKinematicsTwoDOF(q_x[i+1], X_d_tem);
        X_d.push_back(X_d_tem);
    }

    vector<vector<double>> X_d_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < X_d.size(); i++)
        {
            tem.push_back(X_d[i][j]);
        }
        X_d_plot.push_back(tem);
    }




    plt::figure_size(1200, 780);
    for (int i = 0; i < 2; i++)
    {
        string X0_string, X_string, X_d_string;
        X0_string = "X0" + to_string(i+1);
        X_string = "X" + to_string(i+1);
        X_d_string = "X_d" + to_string(i+1);
        
        plt::named_plot(X0_string, X0_plot[i], "--");
        plt::named_plot(X_string, X_plot[i]);
        plt::named_plot(X_d_string, X_d_plot[i]);
    }
    

    plt::xlabel("Time (us)");
    plt::ylabel("Position (m)");
    plt::title("Cartesian Position");
    plt::legend();
    plt::save("CartesianPosition.pdf");
}

void controller_2::plotGeneratedTrajectory()
{
     // plot tau_star
    vector<vector<double>> X0_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < X0.size(); i++)
        {
            tem.push_back(X0[i][j]);
        }
        X0_plot.push_back(tem);
    }

    vector<vector<double>> q0_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i = 0; i < q0.size(); i++)
        {
            tem.push_back(q0[i][j]);
        }
        q0_plot.push_back(tem);
    }

    plt::figure_size(1200, 780);
    for (int i = 0; i < 2; i++)
    {
        string X0_string, q0_string;
        X0_string = "X0" + to_string(i+1);
        q0_string = "q0" + to_string(i+1);
        plt::named_plot(X0_string, X0_plot[i]);
        plt::named_plot(q0_string, q0_plot[i]);

    }
    
    plt::xlabel("Time (us)");
    plt::ylabel("Position (m)");
    plt::title("Cartesian Position");
    plt::legend();
    plt::save("GeneratedTrajectory.pdf");

}


void controller_2::plot_u_x_star()
{
    // plot q_x_star
    vector<vector<double>> u_x_star_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < u_x_star.size(); i++)
        {
            tem.push_back(u_x_star[i][j]);
        }
        u_x_star_plot.push_back(tem);
    }

    plt::figure_size(1200, 780);

    for (int i = 0; i < 2; i++)
    {
        string u_x_star_string;
        u_x_star_string = "u_x_star" + to_string(i+1);
        plt::named_plot(u_x_star_string, u_x_star_plot[i]);
        // plt::named_plot(q_x_string, q_x_plot[i],"--" );
        
    }
    
    plt::legend();
    plt::xlabel("Time (us)");
    plt::ylabel("Position (rad)");
    plt::title("u_x_star");
    // plt::xlim(0, 100);
    plt::save("result_u_x_star.pdf");
}

void controller_2::plot_q_x_star()
{
    vector<vector<double>> q_x_star_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < q_x_star.size(); i++)
        {
            tem.push_back(q_x_star[i][j]);
        }
        q_x_star_plot.push_back(tem);
    }

    plt::figure_size(1200, 780);

    for (int i = 0; i < 2; i++)
    {
        string q_x_star_string;
        q_x_star_string = "q_x_star" + to_string(i+1);
        plt::named_plot(q_x_star_string, q_x_star_plot[i]);
    }

    plt::legend();
    plt::xlabel("Time (us)");
    plt::ylabel("Position (rad)");
    plt::title("q_x_star");
    plt::save("result_q_x_star.pdf");
}


void controller_2::plot_q_star()
{
    vector<vector<double>> q_star_plot;
    for (int j = 0; j < 2; j++)
    {
        vector<double> tem;
        for (int i =0; i < q_star.size(); i++)
        {
            tem.push_back(q_star[i][j]);
        }
        q_star_plot.push_back(tem);
    }


    plt::figure_size(1200, 780);

    for (int i = 0; i < 2; i++)
    {
        string q_star_string;
        q_star_string = "q_star" + to_string(i+1);
        plt::named_plot(q_star_string, q_star_plot[i]);
        // plt::named_plot(q_x_string, q_x_plot[i],"--" );
    }
    
    plt::legend();
    plt::xlabel("Time (us)");
    plt::ylabel("Position (rad)");
    plt::title("q_star");
    // plt::xlim(0, 100);
    plt::save("result_q_star.pdf");
}



void controller_2::saveData()
{
    ofstream outfile_q0("1-q0.txt", ios::out);
    ofstream outfile_q("2-q.txt", ios::out);
    ofstream outfile_q_x("3-q_x.txt", ios::out);

    ofstream outfile_tau("4-tau.txt", ios::out);
    ofstream outfile_tau_star("5-tau_star.txt", ios::out);
    ofstream outfile_f_ext("6-f_ext.txt", ios::out);

    ofstream outfile_X0("7-X0.txt", ios::out);
    ofstream outfile_X("8-X.txt", ios::out);
    ofstream outfile_X_d("9-X_d.txt", ios::out);

    ofstream outfile_JointForce("10-ExternalJointForce.txt", ios::out);
    ofstream outfile_F_ml("11-F_ml.txt", ios::out);

    for (int i = 0; i < count_total; i++)
    {
        outfile_JointForce << tau_ext[i][0] << " "  << tau_ext[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_F_ml << f_e_ML[i][0] << " " << f_e_ML[i][1] << '\n';
    }


    for (int i = 0; i < count_total; i++)
    {
        outfile_q0 << q0[i][0] << " "  << q0[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_q << q[i][0] << " "  << q[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_q_x << q_x[i][0] << " "  << q_x[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_tau << tau[i][0] << " "  << tau[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_tau_star << tau_star[i][0] << " "  << tau_star[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_f_ext << f_ext[i][0] << " "  << f_ext[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_X0 << X0[i][0] << " "  << X0[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_X << X[i][0] << " "  << X[i][1] << '\n';
    }

    for (int i = 0; i < count_total; i++)
    {
        outfile_X_d << X_d[i][0] << " "  << X_d[i][1] << '\n';
    }

    outfile_q0.close();
    outfile_q.close();
    outfile_q_x.close();
    outfile_tau.close();
    outfile_tau_star.close();
    outfile_f_ext.close();
    outfile_X0.close();
    outfile_X.close();
    outfile_X_d.close();


}




controller_2::~controller_2()
{

}


/**
 * @brief
 * @file
 * @author
 * @version
 * @date
 * @param
 * @return
 * @returns
 * @note
 * @exception
 * @property
*/