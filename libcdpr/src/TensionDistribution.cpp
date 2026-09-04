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
    else if ((tau.Min()<Idx_and_lim.CTL_for_TD(1)) && (tau.Max()>Idx_and_lim.CTL_for_TD(2)))
    {
      double maxdiff=tau.Max()-Idx_and_lim.CTL_for_TD(2);
      double mindiff=Idx_and_lim.CTL_for_TD(1)-tau.Min();

      if (maxdiff>mindiff)
      {
        Idx_and_lim.indices_to_set(1)=tau.MaxIdx();
        Idx_and_lim.limits_to_set(1)=Idx_and_lim.CTL_for_TD(2);
      } else
      {
        Idx_and_lim.indices_to_set(1)=tau.MinIdx();
        Idx_and_lim.limits_to_set(1)=Idx_and_lim.CTL_for_TD(1);
      }
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
    else if ((tau.Min()<Idx_and_lim.CTL_for_TD(1)) && (tau.Max()>Idx_and_lim.CTL_for_TD(2)))
    {
      double maxdiff=tau.Max()-Idx_and_lim.CTL_for_TD(2);
      double mindiff=Idx_and_lim.CTL_for_TD(1)-tau.Min();

      if (maxdiff>mindiff)
      {
        Idx_and_lim.indices_to_set(2)=tau.MaxIdx();
        Idx_and_lim.limits_to_set(2)=Idx_and_lim.CTL_for_TD(2);
      } else
      {
        Idx_and_lim.indices_to_set(2)=tau.MinIdx();
        Idx_and_lim.limits_to_set(2)=Idx_and_lim.CTL_for_TD(1);
      }
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
bool GF::updateCablesTensionGF(RobotVars& vars)
  {

    MatrixXd<6,8> A =vars.geom_jacobian_l.Transpose();
    MatrixXd<6,2> Jc=A.GetCols<2>(2); //hard coded; tension controlled cables 2 and 3
    MatrixXd<6,6> Jd;
    Jd.SetCol(1,A.GetCol(1));
    Jd.SetBlock(1,2,A.GetCols<5>(4));
    MatrixXd<2,2> eye2;
    eye2.SetIdentity();
    N.SetBlock(1,1,-Inverse(Jd)*Jc);
    N.SetBlock(7,1,eye2);

   // external loads must be updated in cotroller
    MatrixXd<8,1> fp;
    fp.SetBlock(1,1,Inverse(Jd)*vars.platform.ext_load);
    fp(7,1)=0;
    fp(8,1)=0;
    MatrixXd<8,1> qmax=tau_max-fp;
    MatrixXd<8,1> qmin=tau_min-fp;
    MatrixXd<1,2> ni=N.GetRow(ii);
    MatrixXd<1,2> nj=N.GetRow(jj);
    MatrixXd<2,2> NIJ;
    NIJ.SetRow(1,ni);
    NIJ.SetRow(2,nj);
    MatrixXd<2,1> Q;
    Q.SetRow(1,qmin(ii));
    Q.SetRow(2,qmin(jj));
    MatrixXd<2,1> fc = Inverse(NIJ)*Q;
  //  Polygon=toArmaMat_generic(fc);
    Polygon =arma::mat(fc.Data(), 1, 2, true).t();

    MatrixXd<1, 8> In;
    In.SetZero();

    for (uint ij=1; ij<9 ; ij++)
    {
      MatrixXd<1, 1> first=N.GetRow(ij)*fc-qmin(ij);
      MatrixXd<1, 1> second=N.GetRow(ij)*fc-qmax(ij);

      if ( (std::abs(first(1,1))<eps2) ||  (std::abs(second(1,1))<eps2) || ((first(1,1)>0)&& (second(1,1)<0)) )
      {
        In(ij)=ij;
      } else
      {
        In(ij)=0;
      }
     }
    MatrixXd<2,1> vf=fc;

    int aa=1;
    cnt=0;

    while (aa==1)
    {
      ni=N.GetRow(ii);
      nj=N.GetRow(jj);
      MatrixXd<1,2> ni_p1;
      ni_p1(1,1)=ni(2);
      ni_p1(1,2)=-ni(1);
      MatrixXd<1,2> ni_p2;
      ni_p2(1,1)=-ni(2);
      ni_p2(1,2)=ni(1);

      MatrixXd<1, 1> cond1=nj*fc-qmin(jj);
      MatrixXd<1, 1> cond2=nj*fc-qmax(jj);

      MatrixXd<1,2> ni_p;
      MatrixXd<1, 1> nj_nip1=nj*ni_p1.Transpose();
      if ((std::abs(cond1(1,1)))<eps2)
      {
        if ((nj_nip1(1,1))>=0)
        {
          ni_p=ni_p1;
        }
        else
        {
          ni_p=ni_p2;
        }
      }
      else if ((std::abs(cond2(1,1)))<eps2)
      {
        if ((nj_nip1(1,1))<=0)
        {
          ni_p=ni_p1;
        }
        else
        {
          ni_p=ni_p2;
        }
      }
      else
      {
        printColor('r', "Error1: no condition true ");
      }

      MatrixXd<1,8> alpha;

      alpha = calcalphas(fc, qmin, qmax, ni_p);
      MatrixXd<1,8> ralpha;
      ralpha= roundalphas(alpha,6);


      double al =alpha.Max();

      for (uint idx=1;idx<9;idx++)
      {
        if (ralpha(idx)>0)
        {
          if (alpha(idx)<al)
          {
            al=alpha(idx);
          }
        }
      }

      uint ll;

      if ( (al==alpha.Max()) && (ralpha(alpha.MaxIdx())<=0) )
      {
        printColor('r', "Error2: no acceptable alpha found ");
      }
      else
      {
        fc=fc+al*ni_p.Transpose();
        Polygon.insert_cols(Polygon.n_cols,arma::mat(fc.Data(), 1, 2, true).t());
        for (uint idx=1;idx<9;idx++)
        {
          if (alpha(idx)==al)
          {
           ll=idx;
          }
        }


        if (!(In(ll)==ll))
        {
          In(ll)=ll;
          vf=fc;
          Polygon =arma::mat(fc.Data(), 1, 2, true).t();
          jj=ii;
          ii=ll;
          aa=1;
        }
        else
        {

          if (Norm(fc-vf)>eps1)
          {
            jj=ii;
            ii=ll;
            aa=1;
          }
          else
          {
            if( (In(1)==1) &&
                (In(2)==2) &&
                (In(3)==3) &&
                (In(4)==4) &&
                (In(5)==5) &&
                (In(6)==6) &&
                (In(7)==7) &&
                (In(8)==8)  )
            {
              Polygon.shed_col(Polygon.n_cols-1);
              MatrixXd<2,1> tau_c= TDBaricenter(Polygon);

              MatrixXd<6,1> beq=vars.platform.ext_load-Jc*tau_c;
              MatrixXd<6,1> tau_prelim=Inverse(Jd)*beq;
              vars.tension_vector=arma::ones<arma::vec>(8)*(maxTension+minTension)/2;
              vars.tension_vector(0)=tau_prelim(1);
              vars.tension_vector(1)=tau_c(1);
              vars.tension_vector(2)=tau_c(2);
              vars.tension_vector(3)=tau_prelim(2);
              vars.tension_vector(4)=tau_prelim(3);
              vars.tension_vector(5)=tau_prelim(4);
              vars.tension_vector(6)=tau_prelim(5);
              vars.tension_vector(7)=tau_prelim(6);
              return true;
            }
            else
            {
              vars.tension_vector=arma::ones<arma::vec>(8)*(maxTension+minTension)/2;
              aa=0;
              return false;
            }
          }
        }
      }

      cnt=cnt+1;
      if (cnt==23)
      {aa=0;}
    }

    vars.tension_vector=arma::ones<arma::vec>(8)*(maxTension+minTension)/2;
    return false;
  };


  MatrixXd<1,8> GF::calcalphas(MatrixXd<2,1> fc_, MatrixXd<8,1> qmin_, MatrixXd<8,1> qmax_, MatrixXd<1,2> ni_p_)
  {
    MatrixXd<1,8> alpha;
    alpha.SetZero();
      for (int k=1; k<9;k++)
      {
        MatrixXd<1,2> nk =N.GetRow(k);
        MatrixXd<1, 1> nk_nip=nk*ni_p_.Transpose();
        MatrixXd<1, 1> cond1_=nk*fc_-qmin_(k);
        MatrixXd<1, 1> cond2_=nk*fc_-qmax_(k);


        if (nk_nip(1,1)>eps1)
        {
          if ( (cond1_(1,1)<0) && !(std::abs(cond1_(1,1))<eps2) )
          {
            alpha(k)=-cond1_(1,1)/nk_nip(1,1);
          }
          else if ( ( (cond1_(1,1)>0) && (cond2_(1,1)<0) ) || (std::abs(cond1_(1,1))<eps2)  )
          {
            alpha(k)=-cond2_(1,1)/nk_nip(1,1);
          }
          else
          {
            alpha(k)=1e16;
          }
        }
        else if ( ( nk_nip(1,1)<0 ) && (std::abs(nk_nip(1,1))>eps1) )
        {
          if ( (cond2_(1,1)>0) && !(std::abs(cond2_(1,1))<eps1) )
          {
            alpha(k)=-cond2_(1,1)/nk_nip(1,1);
          }
          else if ( ( (cond1_(1,1)>0) && (cond2_(1,1)<0) ) || (std::abs(cond2_(1,1))<eps1) )
          {
            alpha(k)=-cond1_(1,1)/nk_nip(1,1);
          }
          else
          {
            alpha(k)=1e16;
          }
        }
        else
        {
          if (k==ii)
          {
            alpha(k)=0;
          }
          else
          {
            alpha(k)=1e16;
          }
        }
      }
      return alpha;
  };


  MatrixXd<1,8> GF::roundalphas(MatrixXd<1,8> alpha_,int decimal)
  {
    double factor=std::pow(10.0,decimal);

    for (uint idx=1; idx<9; idx++)
    {
    alpha_(idx)=std::round(alpha_(idx)*factor)/factor;
    }
    return alpha_;
  };

  MatrixXd<2,1> GF::TDBaricenter(arma::mat Polygon_)
  {
    MatrixXd<2,1> tau_c;
    uint number_vertices_=Polygon_.n_cols;
    arma::rowvec wi(number_vertices_);
    for (uint i=0; i<number_vertices_;i++)
    {
      if (i==0)
      {
        double norm1= arma::norm(Polygon_.col(i)-Polygon_.col(number_vertices_-1));
        double norm2= arma::norm(Polygon_.col(i)-Polygon_.col(i+1));

        if (norm(Polygon_.col(i))==0)
        {
          wi(i)=0;
        }
         else
        {
           wi(i)=(norm1+norm2)/norm(Polygon_.col(i));
        }
      }
      else if (i==number_vertices_-1)
      {
        double norm1= arma::norm(Polygon_.col(i)-Polygon_.col(number_vertices_-2));
        double norm2= arma::norm(Polygon_.col(i)-Polygon_.col(0));
        if (norm(Polygon_.col(i))==0)
        {
          wi(i)=0;
        }
        else
        {
          wi(i)=(norm1+norm2)/norm(Polygon_.col(i));
        }
      }
      else
      {
        double norm1= arma::norm(Polygon_.col(i)-Polygon_.col(i-1));
        double norm2= arma::norm(Polygon_.col(i)-Polygon_.col(i+1));
        if (norm(Polygon_.col(i))==0)
        {
          wi(i)=0;
        }
        else
        {
          wi(i)=(norm1+norm2)/norm(Polygon_.col(i));
        }
      }
     }

    arma::vec nom ={0,0};
     double denom=0;
    for (uint i=0; i<number_vertices_;i++)
    {
      nom=nom+wi(i)*Polygon_.col(i);
      denom=denom+wi(i);
    }

        arma::vec lambda=nom/denom;
    tau_c(1)=lambda(0);
    tau_c(2)=lambda(1);
    return tau_c;
  };





    // ============================================================================
    // TurnAround tension-distribution implementation (Gouttefarde et al., 2015)
    // Implemented by Nicolas Testard
    // ============================================================================

         // /!\ For mode 5, robot kinematic must have been updated before



  namespace {

  constexpr unsigned int kTurnaroundCableCount = 8;
  constexpr unsigned int kTurnaroundMaxVertices = 16; // 2*m
  constexpr unsigned int kTurnaroundMaxMoves = 22;    // 3*m-p, with m=8, p=2
  constexpr double kTurnaroundTol = 1e-6;

  inline double RowTimesLambda(const MatrixXd<8,2>& N,
                               const unsigned int row,
                               const MatrixXd<2,1>& lambda)
  {
    return N(row,1)*lambda(1) + N(row,2)*lambda(2);
  }

  inline double RowTimesDirection(const MatrixXd<8,2>& N,
                                  const unsigned int row,
                                  const MatrixXd<2,1>& direction)
  {
    return N(row,1)*direction(1) + N(row,2)*direction(2);
  }

  bool SolveTwoColumns(const MatrixXd<2,1>& c1,
                       const MatrixXd<2,1>& c2,
                       const MatrixXd<2,1>& rhs,
                       MatrixXd<2,1>& x)
  {
    const double det = c1(1)*c2(2) - c2(1)*c1(2);
    if (std::abs(det) <= kTurnaroundTol)
      return false;

    x(1) = (rhs(1)*c2(2) - c2(1)*rhs(2)) / det;
    x(2) = (c1(1)*rhs(2) - rhs(1)*c1(2)) / det;
    return true;
  }

  bool IntersectConstraintLines(const LambdaSpaceTurnaround& space,
                                const unsigned int i,
                                const unsigned int j,
                                const double bi,
                                const double bj,
                                MatrixXd<2,1>& lambda)
  {
    const double det = space.N(i,1)*space.N(j,2)
    - space.N(j,1)*space.N(i,2);
    if (std::abs(det) <= kTurnaroundTol)
      return false;

    lambda(1) = (space.N(j,2)*bi - space.N(i,2)*bj) / det;
    lambda(2) = (space.N(i,1)*bj - space.N(j,1)*bi) / det;
    return true;
  }

  unsigned int CountBoundaryEqualities(const MatrixXd<8,1>& Nlambda,
                                       const MatrixXd<8,1>& boundary)
  {
    unsigned int count = 0;
    for (unsigned int k = 1; k <= kTurnaroundCableCount; ++k)
    {
      if (std::abs(Nlambda(k) - boundary(k)) <= kTurnaroundTol)
        ++count;
    }
    return count;
  }

  void SetConstraintStatesNonMultiple(const LambdaSpaceTurnaround& space,
                                      const MatrixXd<8,1>& Nlambda,
                                      const unsigned int i,
                                      const int type_i,
                                      const unsigned int j,
                                      const int type_j,
                                      int constraint_state[9])
  {
    constraint_state[i] = type_i;
    constraint_state[j] = type_j;

    for (unsigned int k = 1; k <= kTurnaroundCableCount; ++k)
    {
      if (k == i || k == j)
        continue;

      if (Nlambda(k) < space.qmin(k))
        constraint_state[k] = -2;
      else if (Nlambda(k) > space.qmin(k) && Nlambda(k) < space.qmax(k))
        constraint_state[k] = 0;
      else
        constraint_state[k] = 2;
    }
  }

  bool AllConstraintsFeasible(const int constraint_state[9])
  {
    for (unsigned int k = 1; k <= kTurnaroundCableCount; ++k)
    {
      if (std::abs(constraint_state[k]) > 1)
        return false;
    }
    return true;
  }

  bool LambdaIsFeasible(const LambdaSpaceTurnaround& space,
                        const MatrixXd<2,1>& lambda)
  {
    const MatrixXd<8,1> Nlambda = space.N * lambda;
    for (unsigned int k = 1; k <= kTurnaroundCableCount; ++k)
    {
      if (Nlambda(k) < space.qmin(k) - kTurnaroundTol ||
          Nlambda(k) > space.qmax(k) + kTurnaroundTol)
        return false;
    }
    return true;
  }

  bool FindFirstTurnaroundVertex(const LambdaSpaceTurnaround& space,
                                 MatrixXd<2,1>& lambda,
                                 unsigned int& i_out,
                                 int& type_i_out,
                                 unsigned int& j_out,
                                 int& type_j_out,
                                 int constraint_state[9])
  {
    MatrixXd<8,1> Nlambda;

           // Same first search as the MATLAB implementation: intersections of two
           // minimum-tension lines, rejecting multiple-intersection points.
    for (unsigned int i = 1; i <= kTurnaroundCableCount-1; ++i)
    {
      for (unsigned int j = i+1; j <= kTurnaroundCableCount; ++j)
      {
        if (!IntersectConstraintLines(space, i, j,
                                      space.qmin(i), space.qmin(j), lambda))
          continue;

        Nlambda = space.N * lambda;
        if (CountBoundaryEqualities(Nlambda, space.qmax) == 0 &&
            CountBoundaryEqualities(Nlambda, space.qmin) == 2)
        {
          i_out = i;
          j_out = j;
          type_i_out = -1;
          type_j_out = -1;
          SetConstraintStatesNonMultiple(space, Nlambda,
                                         i, -1, j, -1, constraint_state);
          return true;
        }
      }
    }

           // If all min/min intersections are multiple or parallel, try the other
           // three min/max combinations, exactly as in the supplied MATLAB code.
    for (unsigned int i = 1; i <= kTurnaroundCableCount-1; ++i)
    {
      for (unsigned int j = i+1; j <= kTurnaroundCableCount; ++j)
      {
        // i=min, j=max
        if (IntersectConstraintLines(space, i, j,
                                     space.qmin(i), space.qmax(j), lambda))
        {
          Nlambda = space.N * lambda;
          if (CountBoundaryEqualities(Nlambda, space.qmax) == 1 &&
              CountBoundaryEqualities(Nlambda, space.qmin) == 1)
          {
            i_out = i;
            j_out = j;
            type_i_out = -1;
            type_j_out = 1;
            SetConstraintStatesNonMultiple(space, Nlambda,
                                           i, -1, j, 1, constraint_state);
            return true;
          }
        }

               // i=max, j=min
        if (IntersectConstraintLines(space, i, j,
                                     space.qmax(i), space.qmin(j), lambda))
        {
          Nlambda = space.N * lambda;
          if (CountBoundaryEqualities(Nlambda, space.qmax) == 1 &&
              CountBoundaryEqualities(Nlambda, space.qmin) == 1)
          {
            i_out = i;
            j_out = j;
            type_i_out = 1;
            type_j_out = -1;
            SetConstraintStatesNonMultiple(space, Nlambda,
                                           i, 1, j, -1, constraint_state);
            return true;
          }
        }

               // i=max, j=max
        if (IntersectConstraintLines(space, i, j,
                                     space.qmax(i), space.qmax(j), lambda))
        {
          Nlambda = space.N * lambda;
          if (CountBoundaryEqualities(Nlambda, space.qmax) == 2 &&
              CountBoundaryEqualities(Nlambda, space.qmin) == 0)
          {
            i_out = i;
            j_out = j;
            type_i_out = 1;
            type_j_out = 1;
            SetConstraintStatesNonMultiple(space, Nlambda,
                                           i, 1, j, 1, constraint_state);
            return true;
          }
        }
      }
    }

    return false;
  }

  bool KKTVertexL1(const LambdaSpaceTurnaround& space,
                   const unsigned int i,
                   const int type_i,
                   const unsigned int j,
                   const int type_j,
                   MatrixXd<2,1>& mu)
  {
    MatrixXd<8,1> ones;
    for (unsigned int k = 1; k <= kTurnaroundCableCount; ++k)
      ones(k) = 1.0;
    const MatrixXd<2,1> cc = space.N.Transpose() * ones;

    MatrixXd<2,1> c1;
    MatrixXd<2,1> c2;
    c1(1) = -type_i * space.N(i,1);
    c1(2) = -type_i * space.N(i,2);
    c2(1) = -type_j * space.N(j,1);
    c2(2) = -type_j * space.N(j,2);
    return SolveTwoColumns(c1, c2, cc, mu);
  }

  bool KKTVertexL2(const LambdaSpaceTurnaround& space,
                   const unsigned int i,
                   const int type_i,
                   const unsigned int j,
                   const int type_j,
                   const MatrixXd<2,1>& lambda,
                   MatrixXd<2,1>& mu)
  {
    MatrixXd<2,1> c1;
    MatrixXd<2,1> c2;
    c1(1) = -type_i * space.N(i,1);
    c1(2) = -type_i * space.N(i,2);
    c2(1) = -type_j * space.N(j,1);
    c2(2) = -type_j * space.N(j,2);
    return SolveTwoColumns(c1, c2, lambda, mu);
  }

  bool KKTPointOnEdgeL2(const LambdaSpaceTurnaround& space,
                        const unsigned int i,
                        const int type_i,
                        MatrixXd<2,1>& lambda)
  {
    // Same KKT sign test as in the MATLAB implementation.
    if (!((type_i == 1 && -space.qmax(i) >= -kTurnaroundTol) ||
          (type_i == -1 && space.qmin(i) >= -kTurnaroundTol)))
      return false;

    const double ni2 = space.N(i,1)*space.N(i,1)
                       + space.N(i,2)*space.N(i,2);
    if (ni2 <= kTurnaroundTol)
      return false;

    double factor = 0.0;
    if (type_i == 1)
      factor = space.qmax(i) / ni2; // to see if there should be minus that
    else
      factor = space.qmin(i) / ni2;

    lambda(1) = factor * space.N(i,1);
    lambda(2) = factor * space.N(i,2);
    return LambdaIsFeasible(space, lambda);
  }

  bool MultipleIntersectionCoefficients(const LambdaSpaceTurnaround& space,
                                        const unsigned int i,
                                        const int type_i,
                                        const unsigned int j,
                                        const int type_j,
                                        const unsigned int k,
                                        const int type_k,
                                        MatrixXd<2,1>& coef)
  {
    MatrixXd<2,1> c1;
    MatrixXd<2,1> c2;
    MatrixXd<2,1> rhs;

    c1(1) = type_i * space.N(i,1);
    c1(2) = type_i * space.N(i,2);
    c2(1) = type_j * space.N(j,1);
    c2(2) = type_j * space.N(j,2);
    rhs(1) = type_k * space.N(k,1);
    rhs(2) = type_k * space.N(k,2);

    return SolveTwoColumns(c1, c2, rhs, coef);
  }

  bool ComputeCentroidFromOrderedPolygon(const LambdaSpaceTurnaround& space,
                                         MatrixXd<2,1>& lambda)
  {
    if (space.vertex_count < 3)
      return false;

    double sum_d = 0.0;
    double sum_e = 0.0;
    double sum_f = 0.0;

    for (unsigned int i = 1; i <= space.vertex_count; ++i)
    {
      const unsigned int j = (i == space.vertex_count) ? 1 : i+1;
      const double d = space.polygon(1,i)*space.polygon(2,j)
                       - space.polygon(1,j)*space.polygon(2,i);
      sum_d += d;
      sum_e += (space.polygon(1,i)+space.polygon(1,j))*d;
      sum_f += (space.polygon(2,i)+space.polygon(2,j))*d;
    }

    const double area = 0.5*sum_d;
    if (std::abs(area) <= kTurnaroundTol)
      return false;

    lambda(1) = sum_e/(6.0*area);
    lambda(2) = sum_f/(6.0*area);
    return true;
  }

  bool ComputeWeightedBarycenterFromOrderedPolygon(
    const LambdaSpaceTurnaround& space,
    MatrixXd<2,1>& lambda)
  {
    if (space.vertex_count < 3)
      return false;

    MatrixXd<2,1> numerator;
    numerator.SetZero();
    double denominator = 0.0;

    for (unsigned int i = 1; i <= space.vertex_count; ++i)
    {
      const unsigned int iprev = (i == 1) ? space.vertex_count : i-1;
      const unsigned int inext = (i == space.vertex_count) ? 1 : i+1;

      const MatrixXd<2,1> vi = space.polygon.GetCol(i);
      const double weighted_sum = Norm(vi-space.polygon.GetCol(iprev))
                                  + Norm(vi-space.polygon.GetCol(inext));
      const double wi = weighted_sum/Norm(vi);
      numerator += wi*vi;
      denominator += wi;
    }

    lambda = numerator/denominator;
    return true;
  }

  /**
   * Compute the affine directional-geometric-stiffness gradient used by
   * TDA_Kmax2.  The optimized direction is the global +Y translation:
   *
   *     e = [0 1 0 0 0 0]^T.
   *
   * For a fixed pose, the geometric stiffness due to cable tensions is
   * affine in lambda because tau = tp + N*lambda.  The constant term due
   * to tp does not affect the maximizer, so only
   *
   *     K1 = -sum_i N(i,1) Gi,
   *     K2 = -sum_i N(i,2) Gi
   *
   * are required.  The linear objective is
   *
   *     kY(lambda) = constant + a1*lambda1 + a2*lambda2,
   *
   * with a1 = e^T K1 e and a2 = e^T K2 e.
   *
   * Geometry correspondence with the supplied MATLAB code:
   *   r_i                 <-> Q*Bi
   *   cable.pos_PD_glob   <-> r_i
   *   cable.pos_DA_glob   <-> Ai - (X(1:3)+r_i)
   *
   * Hence ||pos_DA_glob|| is the straight anchor-to-platform distance
   * used as l0 in the elementary stiffness matrix Gi.
   */
  bool ComputeDirectionalStiffnessGradientY(
    const RobotVars& vars,
    const LambdaSpaceTurnaround& space,
    double& a1,
    double& a2)
  {
    if (vars.cables.size() < kTurnaroundCableCount)
      return false;

    MatrixXd<6,6> K1;
    MatrixXd<6,6> K2;
    K1.SetZero();
    K2.SetZero();

    MatrixXd<3,3> I3;
    I3.SetIdentity();

    for (unsigned int i = 1; i <= kTurnaroundCableCount; ++i)
    {
      // std::vector is 0-based, grabnum matrices are 1-based.
      const CableVars& cable = vars.cables[i-1];

             // r = Q*Bi in the MATLAB notation.
      const Vector3d r = cable.pos_PD_glob;

             // cable_vec = Ai - Bb in the MATLAB notation.
      const Vector3d cable_vec = cable.pos_DA_glob;
      const double l0 = Norm(cable_vec);
      if (l0 <= kTurnaroundTol)
        return false;

      const Vector3d u = cable_vec/l0;

      const MatrixXd<3,3> b_hat = Skew(r);
      const MatrixXd<3,3> u_hat = Skew(u);
      const MatrixXd<3,3> uuT = u*u.Transpose();
      const MatrixXd<3,3> projector = I3-uuT;

      const MatrixXd<3,3> A11 = -(1.0/l0)*projector;
      const MatrixXd<3,3> A12 =  (1.0/l0)*(projector*b_hat);
      const MatrixXd<3,3> A21 = -(1.0/l0)*(b_hat*projector);
      const MatrixXd<3,3> A22 =
        (u_hat + (1.0/l0)*(b_hat*projector))*b_hat;

      MatrixXd<6,6> Gi;
      Gi.SetBlock(1,1,A11);
      Gi.SetBlock(1,4,A12);
      Gi.SetBlock(4,1,A21);
      Gi.SetBlock(4,4,A22);

      K1 = K1-space.N(i,1)*Gi;
      K2 = K2-space.N(i,2)*Gi;
    }

           // e = [0 1 0 0 0 0]^T, therefore e^T*K*e = K(2,2).
    a1 = K1(2,2);
    a2 = K2(2,2);
    return true;
  }


  /**
   * Maximize the linear directional-stiffness objective over the ordered
   * feasible polygon.  A linear objective on a convex polygon reaches its
   * maximum at at least one vertex, so the MATLAB vertex search is exact.
   */
  bool ComputeMaxDirectionalStiffnessFromOrderedPolygon(
    const LambdaSpaceTurnaround& space,
    const double a1,
    const double a2,
    MatrixXd<2,1>& lambda)
  {
    if (space.vertex_count == 0)
      return false;

    unsigned int best_index = 1;
    double best_value = a1*space.polygon(1,1)
                        + a2*space.polygon(2,1);

    for (unsigned int i = 2; i <= space.vertex_count; ++i)
    {
      const double value = a1*space.polygon(1,i)
      + a2*space.polygon(2,i);
      if (value > best_value)
      {
        best_value = value;
        best_index = i;
      }
    }

    lambda = space.polygon.GetCol(best_index);
    return true;
  }


  /*
bool ComputeDirectionalStiffnessGradientY(
  const RobotVars& vars,
  const LambdaSpaceTurnaround& space,
  double& a1,
  double& a2)
{
  if (vars.cables.size() < kTurnaroundCableCount)
    return false;

 // Full affine active-stiffness matrices:
 //
 // K_a(lambda) = K0 + lambda1*K1 + lambda2*K2
 //MatrixXd<6,6> K0;
 MatrixXd<6,6> K1;
 MatrixXd<6,6> K2;

 //K0.SetZero();
 K1.SetZero();
 K2.SetZero();

 // R maps platform-frame coordinates to global coordinates.
 const MatrixXd<3,3> R  = vars.platform.rot_mat;
 const MatrixXd<3,3> Rt = R.Transpose();

 // 6D block rotation:
 //
 // R6 = [ R  0
 //        0  R ]
 //
 // Used to express the elementary stiffness matrix globally.
 MatrixXd<6,6> R6;
 R6.SetZero();
 R6.SetBlock<3,3>(1,1,R);
 R6.SetBlock<3,3>(4,4,R);

 for (unsigned int i = 1;
      i <= kTurnaroundCableCount;
      ++i)
 {
   // std::vector is 0-based,
   // grabnum matrices are 1-based.
   const CableVars& cable = vars.cables[i-1];


   // ============================================================
   // 1. Geometry expressed in the platform frame
   // ============================================================

   // d_i = P -> D_i
   //
   // D_i is the pulley center, fixed on the moving platform.
   const Vector3d d =
     Rt * cable.pos_PD_glob;

   // q_i = D_i -> A_i
   //
   // A_i is fixed on the base.
   //
   // cable.pos_DA_glob = A_i - D_i
   const Vector3d q =
     Rt * cable.pos_DA_glob;

   // h_i = P -> A_i
   //
   // h_i = d_i + q_i
   const Vector3d h = q + d;


   // Pulley/cable versors.
   //
   // In the IRMA kinematics these are stored in the
   // platform/pulley local frame.
   const Vector3d u = cable.vers_u;
   const Vector3d w = cable.vers_w;
   const Vector3d n = cable.vers_n;
   const Vector3d t = cable.vers_t;


   // s_i = u_i^T q_i
   const double s = Dot(u,q);

   // L_i = || B_i A_i ||
   //
   // Length of the straight tangent cable segment.
   const double L =
     Norm(cable.pos_BA_glob);

   if (std::abs(s) <= kTurnaroundTol ||
       L <= kTurnaroundTol)
   {
     return false;
   }


   const double sin_psi =
     std::sin(cable.tan_ang);

   const double cos_psi =
     std::cos(cable.tan_ang);


   // ============================================================
   // 2. Recover pulley radius from RobotVars
   // ============================================================

   // From the zero-order kinematics:
   //
   // pos_BA =
   //   pos_DA - r * R * (u + n)
   //
   // therefore:
   //
   // R^T(pos_DA-pos_BA) = r (u+n)
   //
   // This lets us recover r without changing the
   // TensionDistribution API to pass RobotParams.

   const Vector3d u_plus_n =
     u + n;

   const Vector3d pulley_offset_loc =
     Rt * (cable.pos_DA_glob -
           cable.pos_BA_glob);

   const double radius_den =
     Dot(u_plus_n,u_plus_n);

   if (radius_den <= kTurnaroundTol)
     return false;

   double radius =
     Dot(pulley_offset_loc,u_plus_n)
     / radius_den;

   if (!std::isfinite(radius) ||
       radius < -kTurnaroundTol)
   {
     return false;
   }

   // Avoid a tiny negative value due only to roundoff.
   if (radius < 0.0)
     radius = 0.0;


   // ============================================================
   // 3. Actual lever arm P -> B_i
   // ============================================================

   // B_i is the actual tangent exit point of the cable.
   //
   // Globally:
   //
   // P->B =
   //   P->D + D->B
   //
   // and
   //
   // D->B =
   //   pos_DA - pos_BA
   //
   // Therefore:
   //
   // beta_i =
   //   R^T [ pos_PD
   //       + pos_DA
   //       - pos_BA ]
   //
   // beta_i is expressed in platform coordinates.

   const Vector3d beta =
     Rt * (cable.pos_PD_glob
           + cable.pos_DA_glob
           - cable.pos_BA_glob);


   // ============================================================
   // 4. Variation of cable tangent direction
   // ============================================================

   // delta(t_i) = C_i delta(q_i)
   //
   // with
   //
   // C_i =
   //   sin(psi_i)/s_i * w_i w_i^T
   // + 1/L_i          * n_i n_i^T
   //
   // The first term is associated with pulley swivel,
   // the second with tangent-angle variation.

   const MatrixXd<3,3> C =
       (sin_psi/s)
       * (w*w.Transpose())

     + (1.0/L)
       * (n*n.Transpose());


   // ============================================================
   // 5. Variation of tangent point B_i
   // ============================================================

   // beta_i =
   //   d_i + r_i (u_i+n_i)
   //
   // therefore
   //
   // delta(beta_i) = E_i delta(q_i)
   //
   // with
   //
   // E_i =
   // r_i [
   //   (1+cos(psi_i))/s_i * w_i w_i^T
   //   - 1/L_i            * t_i n_i^T
   // ]

   const MatrixXd<3,3> E =
     radius *
     (
         ((1.0+cos_psi)/s)
         * (w*w.Transpose())

       - (1.0/L)
         * (t*n.Transpose())
     );


   // ============================================================
   // 6. Auxiliary matrices
   // ============================================================

   // M_i =
   //   [t_i]x E_i
   //   - [beta_i]x C_i

   const MatrixXd<3,3> M =
     Skew(t)*E
     - Skew(beta)*C;

   // mu_i = - beta_i x t_i
   const Vector3d mu =
     -(Skew(beta)*t);


   // ============================================================
   // 7. Full elementary 6x6 stiffness matrix
   // ============================================================

   // Same sign convention as the old implementation.
   //
   // Gi_local =
   //
   // [ -C
   //   -[t]x + C[h]x
   //
   //   M
   //   -M[h]x + [mu]x ]
   //

   const MatrixXd<3,3> G11 =
     -C;

   const MatrixXd<3,3> G12 =
     -Skew(t)
     + C*Skew(h);

   const MatrixXd<3,3> G21 =
     M;

   const MatrixXd<3,3> G22 =
     -M*Skew(h)
     + Skew(mu);


   MatrixXd<6,6> Gi_loc;
   Gi_loc.SetZero();

   Gi_loc.SetBlock<3,3>(1,1,G11);
   Gi_loc.SetBlock<3,3>(1,4,G12);
   Gi_loc.SetBlock<3,3>(4,1,G21);
   Gi_loc.SetBlock<3,3>(4,4,G22);


   // ============================================================
   // 8. Express Gi in the global frame
   // ============================================================

   const MatrixXd<6,6> Gi =
     R6 * Gi_loc * R6.Transpose();


   // ============================================================
   // 9. Affine stiffness matrices in lambda space
   // ============================================================

   // tau = tp + N lambda
   //
   // therefore:
   //
   // K_active(lambda)
   // = K0 + lambda1*K1 + lambda2*K2
   //
   // with
   //
   // K0 = -sum tp_i   Gi
   // K1 = -sum N_i1   Gi
   // K2 = -sum N_i2   Gi

   //K0 = K0 - space.tp(i)*Gi;

   K1 = K1 - space.N(i,1)*Gi;

   K2 = K2 - space.N(i,2)*Gi;
 }


 // ==============================================================
 // 10. Only NOW choose the element to optimize
 // ==============================================================

 // For global +Y translational stiffness:
 //
 // eY = [0 1 0 0 0 0]^T
 //
 // eY^T K eY = K(2,2)
 //
 // K0(2,2) is independent of lambda, so it does
 // not affect the maximizer.

 a1 = K1(2,2);
 a2 = K2(2,2);

  return std::isfinite(a1) &&
         std::isfinite(a2);
}
*/




  bool RunTurnaroundTraversal(LambdaSpaceTurnaround& space,
                              const TurnaroundMode mode,
                              const double stiffness_a1,
                              const double stiffness_a2,
                              MatrixXd<2,1>& lambda_star,
                              int& flag)
  {
    flag = 0;
    space.polygon.SetZero();
    space.vertex_count = 0;
    lambda_star.SetZero();

    const int solution_type = static_cast<int>(mode);
    if (solution_type < 1 || solution_type > 5)
    {
      flag = -2;
      return false;
    }

           // For the minimum 2-norm solution, lambda=0 is immediately optimal when
           // the minimum-norm particular solution tp is feasible.
    if (mode == TurnaroundMode::MinL2)
    {
      MatrixXd<2,1> zero;
      zero.SetZero();
      if (LambdaIsFeasible(space, zero))
      {
        lambda_star = zero;
        flag = 1;
        return true;
      }
    }

    int constraint_state[9] = {0};
    MatrixXd<2,1> vcurrent;
    unsigned int i = 0;
    unsigned int j = 0;
    int ineq_type_i = 0;
    int ineq_type_j = 0;

    if (!FindFirstTurnaroundVertex(space, vcurrent,
                                   i, ineq_type_i,
                                   j, ineq_type_j,
                                   constraint_state))
    {
      flag = -3;
      return false;
    }

    unsigned int number_total = 1;
    MatrixXd<2,1> vfirst = vcurrent;
    bool flag_feas = AllConstraintsFeasible(constraint_state);
    unsigned int number = 0;

           // Initial vertex handling: centroid/barycenter/Kmax store polygon vertices;
           // L1/L2 test KKT optimality immediately when already feasible.
    if (mode == TurnaroundMode::Centroid ||
        mode == TurnaroundMode::WeightedBarycenter ||
        mode == TurnaroundMode::MaxDirectionalStiffnessY ||
        !flag_feas)
    {
      number = 1;
      space.polygon.SetCol(1, vcurrent);
      space.vertex_count = 1;
    }
    else if (mode == TurnaroundMode::MinL1)
    {
      MatrixXd<2,1> mu;
      if (!KKTVertexL1(space, i, ineq_type_i, j, ineq_type_j, mu))
      {
        flag = -5;
        return false;
      }
      if (mu(1) >= -kTurnaroundTol && mu(2) >= -kTurnaroundTol)
      {
        lambda_star = vcurrent;
        flag = 1;
        return true;
      }
      if (mu(2) >= -kTurnaroundTol)
      {
        const unsigned int i_old = i;
        const int type_i_old = ineq_type_i;
        i = j;
        ineq_type_i = ineq_type_j;
        j = i_old;
        ineq_type_j = type_i_old;
      }
    }
    else if (mode == TurnaroundMode::MinL2)
    {
      MatrixXd<2,1> mu;
      if (!KKTVertexL2(space, i, ineq_type_i, j, ineq_type_j,
                       vcurrent, mu))
      {
        flag = -5;
        return false;
      }
      if (mu(1) >= -kTurnaroundTol && mu(2) >= -kTurnaroundTol)
      {
        lambda_star = vcurrent;
        flag = 1;
        return true;
      }
      if (mu(2) >= -kTurnaroundTol)
      {
        const unsigned int i_old = i;
        const int type_i_old = ineq_type_i;
        i = j;
        ineq_type_i = ineq_type_j;
        j = i_old;
        ineq_type_j = type_i_old;
      }

      MatrixXd<2,1> edge_lambda;
      if (KKTPointOnEdgeL2(space, i, ineq_type_i, edge_lambda))
      {
        lambda_star = edge_lambda;
        flag = 1;
        return true;
      }
    }

    int nb_multi = 0;
    int index_multi[9] = {0};
    int ineqtype_multi[9] = {0};
    int nb_multi_previous = 0;
    int index_multi_previous[9] = {0};
    int ineqtype_multi_previous[9] = {0};

    while (number_total <= kTurnaroundMaxMoves)
    {
      // Follow line i in the direction that keeps the inequality associated
      // with line j satisfied (paper Section III-C / MATLAB main loop).
      MatrixXd<2,1> niorth;
      const double det = space.N(i,2)*space.N(j,1)
                         - space.N(i,1)*space.N(j,2);

      if (det > kTurnaroundTol)
      {
        if (ineq_type_j == -1)
        {
          niorth(1) = space.N(i,2);
          niorth(2) = -space.N(i,1);
        }
        else
        {
          niorth(1) = -space.N(i,2);
          niorth(2) = space.N(i,1);
        }
      }
      else if (det < -kTurnaroundTol)
      {
        if (ineq_type_j == -1)
        {
          niorth(1) = -space.N(i,2);
          niorth(2) = space.N(i,1);
        }
        else
        {
          niorth(1) = space.N(i,2);
          niorth(2) = -space.N(i,1);
        }
      }
      else
      {
        flag = -5;
        return false;
      }

             // The opposite inequality of row j gives an initial upper bound on alpha.
      const double tmp_j = RowTimesDirection(space.N, j, niorth);
      if (std::abs(tmp_j) <= kTurnaroundTol)
      {
        flag = -5;
        return false;
      }

      unsigned int k_min = j;
      int ineqtype_min = 0;
      double alpha_min = 0.0;
      if (ineq_type_j == -1)
      {
        ineqtype_min = 1;
        alpha_min = (space.qmax(j)-RowTimesLambda(space.N,j,vcurrent))/tmp_j;
      }
      else
      {
        ineqtype_min = -1;
        alpha_min = (space.qmin(j)-RowTimesLambda(space.N,j,vcurrent))/tmp_j;
      }

             // Save multiple intersections at the current vertex before computing
             // those of the next vertex.
      nb_multi_previous = nb_multi;
      for (int l = 1; l <= nb_multi_previous; ++l)
      {
        index_multi_previous[l] = index_multi[l];
        ineqtype_multi_previous[l] = ineqtype_multi[l];
      }
      nb_multi = 0;
      for (unsigned int k = 1; k <= kTurnaroundCableCount; ++k)
      {
        index_multi[k] = 0;
        ineqtype_multi[k] = 0;
      }

      for (unsigned int k = 1; k <= kTurnaroundCableCount; ++k)
      {
        if (k == i || k == j)
          continue;

        const double tmp = RowTimesDirection(space.N, k, niorth);
        double alpha = alpha_min + 1000.0*kTurnaroundTol;
        int flag_minmax = 0;
        bool alpha_computed = false;

        if (tmp > kTurnaroundTol)
        {
          if (constraint_state[k] == -2)
          {
            alpha = (space.qmin(k)-RowTimesLambda(space.N,k,vcurrent))/tmp;
            flag_minmax = -1;
            alpha_computed = true;
          }
          else if (constraint_state[k] == -1)
          {
            // Current multiple intersection: do not select this line now.
          }
          else if (constraint_state[k] == 0)
          {
            alpha = (space.qmax(k)-RowTimesLambda(space.N,k,vcurrent))/tmp;
            flag_minmax = 1;
            alpha_computed = true;
          }
          else if (constraint_state[k] == 1)
          {
            flag = -6;
            return false;
          }
          // state == 2: no alpha >= 0 intersection to consider.
        }
        else if (tmp < -kTurnaroundTol)
        {
          if (constraint_state[k] == -2)
          {
            // No alpha >= 0 intersection to consider.
          }
          else if (constraint_state[k] == -1)
          {
            flag = -7;
            return false;
          }
          else if (constraint_state[k] == 0)
          {
            alpha = (space.qmin(k)-RowTimesLambda(space.N,k,vcurrent))/tmp;
            flag_minmax = -1;
            alpha_computed = true;
          }
          else if (constraint_state[k] == 1)
          {
            // Current multiple intersection: do not select this line now.
          }
          else if (constraint_state[k] == 2)
          {
            alpha = (space.qmax(k)-RowTimesLambda(space.N,k,vcurrent))/tmp;
            flag_minmax = 1;
            alpha_computed = true;
          }
        }
        // |tmp| <= tol: parallel constraint lines, no candidate intersection.

        if (!alpha_computed)
          continue;

               // The MATLAB transcription contains "<= -tol" at this point; the paper
               // defines a multiple intersection by equal alpha values.  Use <= tol.
        if (std::abs(alpha-alpha_min) <= kTurnaroundTol)
        {
          ++nb_multi;
          index_multi[nb_multi] = static_cast<int>(k);
          ineqtype_multi[nb_multi] = flag_minmax;
        }
        else if (alpha < alpha_min)
        {
          nb_multi = 0;
          alpha_min = alpha;
          k_min = k;
          ineqtype_min = flag_minmax;
        }
      }

      vcurrent = vcurrent + alpha_min*niorth;

             // Returning to vfirst means that a complete turn around the current
             // polygon has been achieved.
      if (Norm(vcurrent-vfirst) <= kTurnaroundTol)
        break;

      ++number_total;

      constraint_state[j] = 0;
      j = i;
      ineq_type_j = ineq_type_i;
      i = k_min;
      ineq_type_i = ineqtype_min;

             // Lines that crossed the previous vertex but are not followed anymore
             // return to the interior state unless they are identical to line i.
      if (nb_multi_previous > 0)
      {
        for (int l = 1; l <= nb_multi_previous; ++l)
        {
          const unsigned int idx = static_cast<unsigned int>(index_multi_previous[l]);
          const double cross = space.N(idx,2)*space.N(i,1)
                               - space.N(idx,1)*space.N(i,2);
          if (std::abs(cross) > kTurnaroundTol)
            constraint_state[idx] = 0;
        }
      }

      bool flag_newpolygon = false;
      const int old_state = constraint_state[k_min];
      constraint_state[k_min] = ineqtype_min;
      if ((ineqtype_min == -1 && old_state == -2) ||
          (ineqtype_min == 1 && old_state == 2))
        flag_newpolygon = true;

      if (nb_multi >= 1)
      {
        for (int l = 1; l <= nb_multi; ++l)
        {
          const unsigned int idx = static_cast<unsigned int>(index_multi[l]);
          const int old_multi_state = constraint_state[idx];
          constraint_state[idx] = ineqtype_multi[l];
          if ((ineqtype_multi[l] == -1 && old_multi_state == -2) ||
              (ineqtype_multi[l] == 1 && old_multi_state == 2))
            flag_newpolygon = true;
        }

               // Remove redundant inequalities at a multiple intersection, following
               // the same geometric test used by the MATLAB implementation.
        for (int l = 1; l <= nb_multi; ++l)
        {
          const unsigned int idx = static_cast<unsigned int>(index_multi[l]);
          MatrixXd<2,1> coef;
          if (!MultipleIntersectionCoefficients(space,
                                                i, ineq_type_i,
                                                j, ineq_type_j,
                                                idx, ineqtype_multi[l],
                                                coef))
          {
            flag = -4;
            return false;
          }

          if (coef(1) < -kTurnaroundTol && coef(2) > kTurnaroundTol)
          {
            j = idx;
            ineq_type_j = ineqtype_multi[l];
          }
          else if (coef(1) < -kTurnaroundTol &&
                   std::abs(coef(2)) <= kTurnaroundTol)
          {
            flag = -4;
            return false;
          }
          else if (coef(1) > kTurnaroundTol && coef(2) < -kTurnaroundTol)
          {
            i = idx;
            ineq_type_i = ineqtype_multi[l];
          }
          else if (std::abs(coef(1)) <= kTurnaroundTol &&
                   coef(2) < -kTurnaroundTol)
          {
            flag = -4;
            return false;
          }
          else if (coef(1) < -kTurnaroundTol && coef(2) < -kTurnaroundTol)
          {
            flag = flag_feas ? -4 : 0;
            return false;
          }
        }
      }

      if (flag_newpolygon)
      {
        vfirst = vcurrent;
        number = 1;
        flag_feas = AllConstraintsFeasible(constraint_state);
      }
      else
      {
        ++number;
      }

      if (mode == TurnaroundMode::Centroid ||
          mode == TurnaroundMode::WeightedBarycenter ||
          mode == TurnaroundMode::MaxDirectionalStiffnessY ||
          !flag_feas)
      {
        if (number > kTurnaroundMaxVertices)
        {
          flag = -77;
          return false;
        }
        space.polygon.SetCol(number, vcurrent);
        space.vertex_count = number;
      }
      else if (mode == TurnaroundMode::MinL1 && flag_feas)
      {
        MatrixXd<2,1> mu;
        if (!KKTVertexL1(space, i, ineq_type_i, j, ineq_type_j, mu))
        {
          flag = -5;
          return false;
        }
        if (mu(1) >= -kTurnaroundTol && mu(2) >= -kTurnaroundTol)
        {
          lambda_star = vcurrent;
          flag = 1;
          return true;
        }
      }
      else if (mode == TurnaroundMode::MinL2 && flag_feas)
      {
        MatrixXd<2,1> mu;
        if (!KKTVertexL2(space, i, ineq_type_i, j, ineq_type_j,
                         vcurrent, mu))
        {
          flag = -5;
          return false;
        }
        if (mu(1) >= -kTurnaroundTol && mu(2) >= -kTurnaroundTol)
        {
          lambda_star = vcurrent;
          flag = 1;
          return true;
        }

        MatrixXd<2,1> edge_lambda;
        if (KKTPointOnEdgeL2(space, i, ineq_type_i, edge_lambda))
        {
          lambda_star = edge_lambda;
          flag = 1;
          return true;
        }
      }
    }

    if (number_total > kTurnaroundMaxMoves)
    {
      flag = -66;
      return false;
    }

    if (!flag_feas)
    {
      flag = 0;
      return false;
    }

    flag = 1;
    if (mode == TurnaroundMode::Centroid)
      return ComputeCentroidFromOrderedPolygon(space, lambda_star);

    if (mode == TurnaroundMode::WeightedBarycenter)
      return ComputeWeightedBarycenterFromOrderedPolygon(space, lambda_star);

    if (mode == TurnaroundMode::MaxDirectionalStiffnessY)
      return ComputeMaxDirectionalStiffnessFromOrderedPolygon(
        space, stiffness_a1, stiffness_a2, lambda_star);

           // For L1/L2 the optimum should have been detected while following the
           // feasible polygon, as in the MATLAB implementation.
    flag = (mode == TurnaroundMode::MinL1) ? -33 : -44;
    return false;
  }

  } // anonymous namespace


  bool InitializeLambdaSpaceTurnaround(const RobotVars& vars,
                                    const double min_tension,
                                    const double max_tension,
                                    LambdaSpaceTurnaround& space)
  {
    space.N.SetZero();
    space.tp.SetZero();
    space.qmin.SetZero();
    space.qmax.SetZero();
    space.polygon.SetZero();
    space.vertex_count = 0;

    if (max_tension <= min_tension)
      return false;

    const MatrixXd<6,8> W = vars.geom_jacobian_l.Transpose();

           // Same null-space construction tested in the MATLAB prototype:
           // dependent columns [1 4 5 6 7 8], free columns [2 3].
    const MatrixXd<6,2> Wc = W.GetCols<2>(2);
    MatrixXd<6,6> Wd;
    Wd.SetCol(1, W.GetCol(1));
    Wd.SetBlock(1,2, W.GetCols<5>(4));

    MatrixXd<6,6> Wd_inv;
    try
    {
      Wd_inv = Inverse(Wd);
    }
    catch (const std::runtime_error& e)
    {
      std::cout << "InitializeLambdaSpaceTurnaround: singular Wd: "
                << e.what() << std::endl;
      return false;
    }

           // First order is [T1 T4 T5 T6 T7 T8 T2 T3].
    MatrixXd<8,2> N_reordered;
    N_reordered.SetBlock(1,1, -(Wd_inv*Wc));
    MatrixXd<2,2> I2;
    I2.SetIdentity();
    N_reordered.SetBlock(7,1,I2);

           // Gram-Schmidt exactly in the order used in the supplied MATLAB code.
    MatrixXd<8,1> n2 = N_reordered.GetCol(2);
    const double norm_n2 = Norm(n2);
    if (norm_n2 <= kTurnaroundTol)
      return false;
    n2 = n2/norm_n2;

    MatrixXd<8,1> n1 = N_reordered.GetCol(1);
    n1 = n1 - Dot(n1,n2)*n2;
    const double norm_n1 = Norm(n1);
    if (norm_n1 <= kTurnaroundTol)
      return false;
    n1 = n1/norm_n1;

    N_reordered.SetCol(1,n1);
    N_reordered.SetCol(2,n2);

           // Restore physical cable order [T1 T2 T3 T4 T5 T6 T7 T8].
    space.N.SetRow(1,N_reordered.GetRow(1));
    space.N.SetRow(2,N_reordered.GetRow(7));
    space.N.SetRow(3,N_reordered.GetRow(8));
    space.N.SetRow(4,N_reordered.GetRow(2));
    space.N.SetRow(5,N_reordered.GetRow(3));
    space.N.SetRow(6,N_reordered.GetRow(4));
    space.N.SetRow(7,N_reordered.GetRow(5));
    space.N.SetRow(8,N_reordered.GetRow(6));

           // Reuse Wd_inv to obtain a particular solution without a second 6x6
           // inversion. In reordered cable coordinates [T1 T4 T5 T6 T7 T8 T2 T3]:
           //   tp0 = [Wd^-1*f ; 0 ; 0].
           // Since space.N is orthonormal, removing the null-space component
           // yields the minimum-norm particular solution tp = W^+*f:
           //   tp = tp0 - N*(N^T*tp0).
    MatrixXd<8,1> tp0_reordered;
    tp0_reordered.SetZero();
    tp0_reordered.SetBlock(1,1, Wd_inv*vars.platform.ext_load);

    MatrixXd<8,1> tp0;
    tp0(1) = tp0_reordered(1);
    tp0(2) = tp0_reordered(7);
    tp0(3) = tp0_reordered(8);
    tp0(4) = tp0_reordered(2);
    tp0(5) = tp0_reordered(3);
    tp0(6) = tp0_reordered(4);
    tp0(7) = tp0_reordered(5);
    tp0(8) = tp0_reordered(6);

    const MatrixXd<2,1> null_component = space.N.Transpose()*tp0;
    space.tp = tp0 - space.N*null_component;

    MatrixXd<8,1> tau_min;
    MatrixXd<8,1> tau_max;
    for (unsigned int k = 1; k <= kTurnaroundCableCount; ++k)
    {
      tau_min(k) = min_tension;
      tau_max(k) = max_tension;
    }

    space.qmin = tau_min-space.tp;
    space.qmax = tau_max-space.tp;
    return true;
  }


  bool UpdateCablesTensionTurnaround(RobotVars& vars,
                                     const TurnaroundMode mode)
  {
    Index_and_limits Idx_and_lim{};
    const double min_tension = Idx_and_lim.CTL_for_TD(1);
    const double max_tension = Idx_and_lim.CTL_for_TD(2);
    const double fallback = 0.5*(min_tension+max_tension);

    LambdaSpaceTurnaround space;
    if (!InitializeLambdaSpaceTurnaround(vars,
                                      min_tension,
                                      max_tension,
                                      space))
    {
      vars.tension_vector = arma::ones<arma::vec>(8)*fallback;
      return false;
    }

    double stiffness_a1 = 0.0;
    double stiffness_a2 = 0.0;
    if (mode == TurnaroundMode::MaxDirectionalStiffnessY)
    {
      if (!ComputeDirectionalStiffnessGradientY(
            vars, space, stiffness_a1, stiffness_a2))
      {
        vars.tension_vector = arma::ones<arma::vec>(8)*fallback;
        return false;
      }
    }

    MatrixXd<2,1> lambda_star;
    int flag = 0;
    if (!RunTurnaroundTraversal(space, mode, stiffness_a1, stiffness_a2, lambda_star, flag))
    {
      vars.tension_vector = arma::ones<arma::vec>(8)*fallback;
      return false;
    }

    const MatrixXd<8,1> tau = space.tp + space.N*lambda_star;
    vars.tension_vector = arma::vec(tau.Data(),8);
    return true;
  }

}

