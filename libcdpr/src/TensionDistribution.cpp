/**
 * @file TensionDistribution.cpp
 * @author Anna Berger
 * @date 13 Nov 2025
 * @brief File containing definitions of functions declared in TensionDistribution.h.
 */

#include "TensionDistribution.h"


namespace grabcdpr {

bool updateCablesTensionDistribution(RobotVars& vars)
{
  MatrixXd<8,1> tau_v;
  Index_and_limits Idx_and_lim{};

  MatrixXd<8,1> tau_m;
  double taumean=0.5*( Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
  tau_m.Fill(std::vector<double>{taumean,taumean,taumean,taumean,taumean,taumean,taumean,taumean});
  MatrixXd<6,8>A =vars.geom_jacobian_l.Transpose();
  Vector6d b=vars.platform.ext_load-A*tau_m;
  MatrixXd<8,6> AT=A.Transpose();
  MatrixXd<6,6> AAT=A*AT;
 // MatrixXd<8,6> MoorePenrose=AT*Inverse(AAT); //try catch block for inverse
  MatrixXd<8,6> MoorePenrose;
  try {
    MoorePenrose=AT*Inverse(AAT);
  }  catch (const std::runtime_error& e) {
    std::cout << "error:" << e.what() << std::endl;
    vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
    return false;
  }


  tau_v=MoorePenrose*b;

  MatrixXd<1,8> row_tau_v=tau_v.Transpose();
  double normatauv=Norm(row_tau_v);

  if (normatauv> 0.5*sqrt(8)*(Idx_and_lim.CTL_for_TD(2)-Idx_and_lim.CTL_for_TD(1)))
    {
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
      return false;
    }

    MatrixXd<8,1>  tau =tau_v+tau_m;


    if ((tau.Min()>=Idx_and_lim.CTL_for_TD(1)) && (tau.Max() <=Idx_and_lim.CTL_for_TD(2)))
    {
      vars.tension_vector=arma::vec(tau.Data(),8,1,true);
     // vars.tension_vector= toArmaMat_generic(tau);
      return true;
    }
    else if (tau.Max()>Idx_and_lim.CTL_for_TD(2))
    {
      Idx_and_lim.indices_to_set(1)=tau.MaxIdx();
      Idx_and_lim.limits_to_set(1)=Idx_and_lim.CTL_for_TD(2);
    }
    else if (tau.Min()<Idx_and_lim.CTL_for_TD(1))
    {
      Idx_and_lim.indices_to_set(1)=tau.MinIdx();
      Idx_and_lim.limits_to_set(1)=Idx_and_lim.CTL_for_TD(1);
    }

// try fixing non acceptable value to limit
    //adjust
    MatrixXd<7,1> tau_v7;
    MatrixXd<7,1> tau_m7;
    tau_m7.Fill(std::vector<double>{taumean,taumean,taumean,taumean,taumean,taumean,taumean});
    MatrixXd<6,7> smallerA;

    uint counter=0;
    for (uint i=1;i<9;++i)
    {
      if (!(i==Idx_and_lim.indices_to_set(1)))
      {
       counter=counter+1;
       smallerA.SetCol(counter,A.GetCol(i));
      }
    }


    b=vars.platform.ext_load-smallerA*tau_m7-A.GetCol(Idx_and_lim.indices_to_set(1))*Idx_and_lim.limits_to_set(1);
    MatrixXd<7,6> smallerAT=smallerA.Transpose();
    MatrixXd<6,6> SA_SAT=smallerA*smallerAT;
   // MatrixXd<7,6> smallerMoorePenrose=smallerAT*Inverse(SA_SAT); //try catch block for inverse
    MatrixXd<7,6> smallerMoorePenrose;
    try {
      smallerMoorePenrose=smallerAT*Inverse(SA_SAT);
    }  catch (const std::runtime_error& e) {
      std::cout << "error:" << e.what() << std::endl;
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
      return false;
    }


    tau_v7=smallerMoorePenrose*b;

    MatrixXd<1,7> row_tau_v7=tau_v7.Transpose();
    double normatauv7=Norm(row_tau_v7);

    //



    if ((normatauv7) > 0.5*sqrt(7)*(Idx_and_lim.CTL_for_TD(2)-Idx_and_lim.CTL_for_TD(1)))
    {
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
      return false;
    }

    MatrixXd<7,1> tau_h =tau_v7+tau_m7;

    tau = insertLimitTension7to8(tau_h, Idx_and_lim.indices_to_set(1),Idx_and_lim.limits_to_set(1));

    if ((tau.Min()>=Idx_and_lim.CTL_for_TD(1)) && (tau.Max()<=Idx_and_lim.CTL_for_TD(2)))
    {
      vars.tension_vector=arma::vec(tau.Data(),8,1,true);
      // vars.tension_vector= toArmaMat_generic(tau);
      return true;
    }
    else if (tau.Max()>Idx_and_lim.CTL_for_TD(2))
    {
      Idx_and_lim.indices_to_set(2)=tau.MaxIdx();
      Idx_and_lim.limits_to_set(2)=Idx_and_lim.CTL_for_TD(2);
    }
    else if (tau.Min()<Idx_and_lim.CTL_for_TD(1))
    {
      Idx_and_lim.indices_to_set(2)=tau.MinIdx();
      Idx_and_lim.limits_to_set(2)=Idx_and_lim.CTL_for_TD(1);
    }

//second try fixing non acceptable value to limit
    //adjust
    MatrixXd<6,1> tau_v6;
    MatrixXd<6,1> tau_m6;
    tau_m6.Fill(std::vector<double>{taumean,taumean,taumean,taumean,taumean,taumean});
    MatrixXd<6,6> smallestA;

    uint idxa =Idx_and_lim.indices_to_set.Max();
    uint idxb = Idx_and_lim.indices_to_set.Min();
    double limitA=Idx_and_lim.limits_to_set(Idx_and_lim.indices_to_set.MaxIdx());
    double limitB=Idx_and_lim.limits_to_set(Idx_and_lim.indices_to_set.MinIdx());

    counter=0;
    for (uint i=1;i<9;++i)
    {
      if (!(i==idxa) && (!(i==idxb)))
      {
        counter=counter+1;
        smallestA.SetCol(counter,A.GetCol(i));
      }
    }


    b=vars.platform.ext_load-smallestA*tau_m6-A.GetCol(idxa)*limitA-A.GetCol(idxb)*limitB;
    MatrixXd<6,6> smallestAT=smallestA.Transpose();
    MatrixXd<6,6> SMA_SMAT=smallestA*smallestAT;
   // MatrixXd<6,6> smallestMoorePenrose=smallestAT*Inverse(SMA_SMAT); //try catch block for inverse
    MatrixXd<6,6> smallestMoorePenrose;
    try {
      smallestMoorePenrose=smallestAT*Inverse(SMA_SMAT);
    }  catch (const std::runtime_error& e) {
      std::cout << "error:" << e.what() << std::endl;
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(2)+Idx_and_lim.CTL_for_TD(1));
      return false;
    }
    tau_v6=smallestMoorePenrose*b;

    MatrixXd<1,6> row_tau_v6=tau_v6.Transpose();
    double normatauv6=Norm(row_tau_v6);



    if (normatauv6 > 0.5*sqrt(6)*(Idx_and_lim.CTL_for_TD(2)-Idx_and_lim.CTL_for_TD(1)))
    {
      vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(1)+Idx_and_lim.CTL_for_TD(2));
      return false;
    }

    MatrixXd<6,1> tau_h1 =tau_v6+tau_m6;
    MatrixXd<7,1> tau_h2;


      tau_h2 = insertLimitTension6to7(tau_h1, idxb,limitB);
      tau = insertLimitTension7to8(tau_h2, idxa,limitA);


      if ((tau.Min()>=Idx_and_lim.CTL_for_TD(1)) && (tau.Max()<=Idx_and_lim.CTL_for_TD(2)))
     {
       vars.tension_vector=arma::vec(tau.Data(),8,1,true);
        //vars.tension_vector= toArmaMat_generic(tau);
       return true;
     }
     else
     {
       vars.tension_vector=arma::ones<arma::vec>(8)*0.5*(Idx_and_lim.CTL_for_TD(1)+Idx_and_lim.CTL_for_TD(2));
       return false;
     }



}

MatrixXd<7,1> insertLimitTension6to7(const MatrixXd<6,1>& tau_h_,
                             const uint& index_to_set,
                             const double& TensionLimit)
{
    MatrixXd<7,1> tauout;
    for (uint i=1;i<8;++i)
    {
      if (i<index_to_set)
      {
        tauout(i,1)=tau_h_(i,1);
      } else if (i==index_to_set)
      {
        tauout(i,1)=TensionLimit;
      } else if (i>index_to_set)
      {
        tauout(i,1)=tau_h_(i-1,1);
      }
    }
      return tauout;
  }


MatrixXd<8,1> insertLimitTension7to8(const MatrixXd<7,1>& tau_h_,
                                  const uint& index_to_set,
                                  const double& TensionLimit)
{

    MatrixXd<8,1> tauout;
    for (uint i=1;i<9;++i)
    {
      if (i<index_to_set)
      {
        tauout(i,1)=tau_h_(i,1);
      } else if (i==index_to_set)
      {
        tauout(i,1)=TensionLimit;
      } else if (i>index_to_set)
      {
        tauout(i,1)=tau_h_(i-1,1);
      }
    }
      return tauout;

}

}
/**
arma::vec calcCableTensionDistribution(const arma::mat& geom_jacobian,
                                       const Vector6d& ext_load,
                                       const arma::vec& limits,
                                       const Index_and_limits& _Idx_and_lim)
{
  int tau_v_dimension=8;
  arma::mat extracted_cols=arma::zeros<arma::mat>(6,2);
  arma::mat A=geom_jacobian;

  if ((!(_Idx_and_lim.set_one_limit)) && (!(_Idx_and_lim.set_second_limit)))
   {
      tau_v_dimension=8;
      A=geom_jacobian;
   }
   else if ((_Idx_and_lim.set_one_limit) && (!(_Idx_and_lim.set_second_limit)))
   {
      tau_v_dimension=7;
      extracted_cols.col(0)=geom_jacobian.col(_Idx_and_lim.index_one_to_set);

      arma::vec mask=arma::zeros<arma::vec>(8);
      mask(_Idx_and_lim.index_one_to_set)=1;

      arma::uvec keepcol=arma::find(mask==0);
      A=geom_jacobian.cols(keepcol);
    }
    else if ((_Idx_and_lim.set_one_limit) && (_Idx_and_lim.set_second_limit))
    {
      tau_v_dimension=6;
      extracted_cols.col(0)=geom_jacobian.col(_Idx_and_lim.index_one_to_set);
      extracted_cols.col(1)=geom_jacobian.col(_Idx_and_lim.index_two_to_set);

      arma::vec mask=arma::zeros<arma::vec>(8);
      mask(_Idx_and_lim.index_one_to_set)=1;
      mask(_Idx_and_lim.index_two_to_set)=1;
      arma::uvec keepcol=arma::find(mask==0);
      A=geom_jacobian.cols(keepcol);
     }
     else
     {
     printColor('r', "ERROR: Cancel_index has unexpected values");
     }


  arma::vec tau_m = arma::ones<arma::vec>(tau_v_dimension)*0.5*(limits(0)+limits(1));
  arma::vec b=toArmaVec(ext_load)-extracted_cols*_Idx_and_lim.limits_to_set-A*tau_m;


  return arma::pinv(A)*b;

}
}
*/

