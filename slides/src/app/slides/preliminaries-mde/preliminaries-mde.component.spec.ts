import { ComponentFixture, TestBed } from '@angular/core/testing';

import { PreliminariesMdeComponent } from './preliminaries-mde.component';

describe('PreliminariesMdeComponent', () => {
  let component: PreliminariesMdeComponent;
  let fixture: ComponentFixture<PreliminariesMdeComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ PreliminariesMdeComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(PreliminariesMdeComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
